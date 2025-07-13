#include "AutoTimeSync.h"
#include <Arduino.h>

namespace mesh {

void AutoTimeSync::processSample(uint32_t sender_timestamp, uint8_t hop_count, uint8_t sender_hash, uint32_t estimated_delay_ms) {
  if (!enabled || !_rtc) return;
  
  const uint8_t max_hops_cached = max_hops;
  const uint32_t max_drift_cached = max_drift;
  
  if (hop_count > max_hops_cached) return;
  
  const uint32_t current_time = _rtc->getCurrentTime();
  
  // Add transmission delay compensation
  const uint32_t delay_secs = (estimated_delay_ms + 999) / 1000;
  uint32_t adjusted_timestamp = sender_timestamp;
  
  if (delay_secs > 0 && sender_timestamp <= (UINT32_MAX - delay_secs)) {
    adjusted_timestamp += delay_secs;
  }
  
  // Validate timestamp
  if (!isValidTimestamp(adjusted_timestamp, current_time, max_drift_cached)) {
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Invalid timestamp rejected: %u (current: %u)", adjusted_timestamp, current_time);
#endif
    return;
  }
  
#if MESH_DEBUG
  MESH_DEBUG_PRINTLN("AutoTimeSync: Processing sample from %02X, hops=%d, delay=%ums", 
                     sender_hash, hop_count, estimated_delay_ms);
#endif
  
  addSample(adjusted_timestamp, hop_count, sender_hash);
  
  if (shouldSync()) {
    performSync();
  }
}

void AutoTimeSync::processForwardedSample(uint32_t sender_timestamp, uint8_t hop_count, uint8_t sender_hash, uint32_t estimated_delay_ms) {
  if (!enabled || !_rtc) return;
  
  const uint8_t max_hops_cached = max_hops;
  const uint32_t max_drift_cached = max_drift;
  
  if (hop_count > max_hops_cached) return;
  
  const uint32_t current_time = _rtc->getCurrentTime();
  
  // Add transmission delay compensation
  const uint32_t delay_secs = (estimated_delay_ms + 999) / 1000;
  uint32_t adjusted_timestamp = sender_timestamp;
  
  if (delay_secs > 0 && sender_timestamp <= (UINT32_MAX - delay_secs)) {
    adjusted_timestamp += delay_secs;
  }
  
  // Validate timestamp
  if (!isValidTimestamp(adjusted_timestamp, current_time, max_drift_cached)) {
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Invalid forwarded timestamp rejected: %u (current: %u)", adjusted_timestamp, current_time);
#endif
    return;
  }
  
  // Track forwarded samples
  if (forwarded_samples_count < UINT32_MAX) {
    forwarded_samples_count++;
  }
  
#if MESH_DEBUG
  MESH_DEBUG_PRINTLN("AutoTimeSync: Processing forwarded sample from %02X, hops=%d, delay=%ums", 
                     sender_hash, hop_count, estimated_delay_ms);
#endif
  
  addSample(adjusted_timestamp, hop_count, sender_hash);
  
  if (shouldSync()) {
    performSync();
  }
}

bool AutoTimeSync::shouldSync() const {
  if (!enabled || !_rtc || sample_count <= 0 || sample_count > MAX_SAMPLES) return false;
  
  const uint8_t min_samples_cached = min_samples;
  
  // Initial sync requires at least 2 samples for reliability
  const int required_samples = (last_sync_time == 0) ? 
                              ((min_samples_cached >= 2) ? min_samples_cached : 2) :
                              min_samples_cached;
  
  if (required_samples <= 0 || sample_count < required_samples) return false;

  const uint32_t current_time = _rtc->getCurrentTime();
  
  // Rate limiting - prevent sync spam
  if (current_time >= MIN_REASONABLE_TIME && 
      last_sync_time > 0 && 
      last_sync_time <= (UINT32_MAX - MIN_SYNC_INTERVAL) &&
      current_time < last_sync_time + MIN_SYNC_INTERVAL) {
    return false;
  }
  
  // Multiple sources required for initial sync
  if (last_sync_time == 0) {
    const int unique_sources = getUniqueSources();
    
    if (sample_count >= 4 && unique_sources < 2) {
#if MESH_DEBUG
      MESH_DEBUG_PRINTLN("AutoTimeSync: Initial sync delayed - need more sources (%d samples from %d sources)", 
                         sample_count, unique_sources);
#endif
      return false;
    }
  }
  
  // Calculate consensus time
  const uint32_t consensus_time = calculateConsensusTime();
  if (consensus_time == 0) return false;
  
  int recent_samples = 0;
  int consensus_count = 0;
  
  for (int i = 0; i < sample_count; i++) {
    if (!isValidSampleIndex(i)) continue;
    
    const bool is_recent = isRecentSample(samples[i].received_at, current_time);
    
    if (is_recent) {
      recent_samples++;
      
      // Account for time elapsed since reception
      uint32_t time_since_reception = 0;
      if (current_time >= samples[i].received_at) {
        time_since_reception = current_time - samples[i].received_at;
      }
      
      uint32_t current_time_estimate = samples[i].timestamp;
      if (time_since_reception > 0 && current_time_estimate <= (UINT32_MAX - time_since_reception)) {
        current_time_estimate += time_since_reception;
      }
      
      // Tolerance varies by sync state
      uint32_t tolerance;
      if (last_sync_time == 0) {
        tolerance = 300;  // 5 minutes for initial sync
      } else {
        const uint32_t sample_age = (current_time >= samples[i].received_at) ? 
                                   (current_time - samples[i].received_at) : 0;
        tolerance = calculateTolerance(sample_age);
      }
      
      const uint32_t abs_deviation = (current_time_estimate >= consensus_time) ? 
                                     (current_time_estimate - consensus_time) : 
                                     (consensus_time - current_time_estimate);
      
      if (abs_deviation <= tolerance) {
        consensus_count++;
      }
      
#if MESH_DEBUG
      MESH_DEBUG_PRINTLN("AutoTimeSync: Sample %d - estimate: %u, consensus: %u, deviation: %u, tolerance: %u, agrees: %s",
                         i, current_time_estimate, consensus_time, abs_deviation, tolerance, 
                         (abs_deviation <= tolerance) ? "YES" : "NO");
#endif
    }
  }
  
#if MESH_DEBUG
  if (last_sync_time == 0) {
    const int unique_sources = getUniqueSources();
    MESH_DEBUG_PRINTLN("AutoTimeSync: Recent samples: %d/%d, consensus: %d, sources: %d (need %d samples, initial sync)", 
                       recent_samples, sample_count, consensus_count, unique_sources, required_samples);
  } else {
    MESH_DEBUG_PRINTLN("AutoTimeSync: Recent samples: %d/%d, consensus: %d (need %d, subsequent sync)", 
                       recent_samples, sample_count, consensus_count, required_samples);
  }
#endif
  
  return (recent_samples >= required_samples && consensus_count >= required_samples);
}

bool AutoTimeSync::performSync() {
  if (!enabled || !_rtc) return false;
  
  const uint32_t consensus_time = calculateConsensusTime();
  if (consensus_time == 0) return false;
  
  const uint32_t rtc_now = _rtc->getCurrentTime();
  
  // Calculate drift
  const uint32_t abs_drift = (consensus_time >= rtc_now) ? 
                            (consensus_time - rtc_now) : 
                            (rtc_now - consensus_time);
  const int32_t drift = (int32_t)(consensus_time - rtc_now);
  
  // Validate consensus time
  if (!isValidTimestamp(consensus_time, rtc_now, max_drift)) {
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Consensus time failed validation: %u (current: %u, drift: %d)", 
                       consensus_time, rtc_now, drift);
#endif
    return false;
  }
  
  // Perform plausibility check if we have prior sync history
  if (last_sync_rtc != 0 && !performPlausibilityCheck(consensus_time, rtc_now)) {
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Consensus time failed plausibility check");
#endif
    return false;
  }
  
  // Skip trivial adjustments
  if (last_sync_time != 0 && abs_drift < 10) {
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Drift minimal, no sync needed: %d seconds", drift);
#endif
    return false;
  }
  
  // Monotonicity + slew rate guard
  if (consensus_time < last_sync_time) {
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Rejecting backwards time adjustment: %u < %u", consensus_time, last_sync_time);
#endif
    return false;  // No backwards time
  }
  
  // Apply slew rate limiting
  uint32_t target_time = consensus_time;
  if (abs_drift > max_slew) {
    if (drift > 0) {
      target_time = rtc_now + max_slew;
    } else {
      target_time = rtc_now - max_slew;
    }
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Slew rate limiting applied: drift=%d, max_slew=%u, target=%u", 
                       drift, max_slew, target_time);
#endif
  }
  
  // Update drift rate calculation
  updateDriftRate(consensus_time, rtc_now);
  
  // Apply the time update
  _rtc->setCurrentTime(target_time);
  last_sync_time = target_time;
  
  // Keep recent samples, remove old ones
  int kept_samples = 0;
  for (int i = 0; i < sample_count; i++) {
    if (isValidSampleIndex(i) && isRecentSample(samples[i].received_at, target_time)) {
      if (kept_samples != i) {
        samples[kept_samples] = samples[i];
      }
      kept_samples++;
    }
  }
  sample_count = kept_samples;
  
  // Reset forwarded counter
  forwarded_samples_count = 0;
  
#if MESH_DEBUG
  if (rtc_now < MIN_REASONABLE_TIME) {
    MESH_DEBUG_PRINTLN("AutoTimeSync: Clock synchronized from unset state, set to %u", target_time);
  } else {
    MESH_DEBUG_PRINTLN("AutoTimeSync: Clock synchronized, drift was %d seconds, applied %d seconds, drift_rate=%.6f", 
                       drift, (int32_t)(target_time - rtc_now), drift_rate);
  }
#endif
  return true;
}

bool AutoTimeSync::isValidTimestamp(uint32_t timestamp, uint32_t current_time, uint32_t max_drift_param) const {
  // Basic sanity check
  if (timestamp < MIN_REASONABLE_TIME || timestamp > MAX_REASONABLE_TIME) {
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Timestamp outside reasonable bounds: %u (valid: %u-%u)", 
                       timestamp, MIN_REASONABLE_TIME, MAX_REASONABLE_TIME);
#endif
    return false;
  }
  
  // Handle unset local clock
  if (current_time < MIN_REASONABLE_TIME) {
    const uint32_t future_limit = 1893456000;  // 2035-01-01
    if (timestamp > future_limit) {
#if MESH_DEBUG
      MESH_DEBUG_PRINTLN("AutoTimeSync: Timestamp too far in future for unset clock: %u (limit: %u)", 
                         timestamp, future_limit);
#endif
      return false;
    }
    
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Accepting timestamp for unset clock: %u", timestamp);
#endif
    return true;
  }
  
  // Initial bootstrap sync with larger tolerance
  if (last_sync_time == 0) {
    const uint32_t abs_diff = (timestamp >= current_time) ? 
                             (timestamp - current_time) : 
                             (current_time - timestamp);
    const int32_t diff = (int32_t)(timestamp - current_time);
    
    if (abs_diff <= MAX_INITIAL_DRIFT) {
#if MESH_DEBUG
      MESH_DEBUG_PRINTLN("AutoTimeSync: Accepting timestamp for initial sync: %u (current: %u, diff: %d seconds)", 
                         timestamp, current_time, diff);
#endif
      return true;
    }
    
    // Handle old firmware with stale time
    const uint32_t aged_threshold = 47304000;  // 18 months
    if (diff > 0 && diff > (int32_t)aged_threshold) {
      if (timestamp <= MAX_REASONABLE_TIME - MAX_FUTURE_ALLOWANCE) {
#if MESH_DEBUG
        MESH_DEBUG_PRINTLN("AutoTimeSync: Accepting timestamp from old firmware (current: %u, timestamp: %u, diff: %d months)", 
                           current_time, timestamp, diff / 2628000);
#endif
        return true;
      }
    }
    
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Rejecting timestamp with excessive drift for initial sync: %d seconds", diff);
#endif
    return false;
  }
  
  // Normal operation - enforce configured drift limit
  const uint32_t abs_diff = (timestamp >= current_time) ? 
                           (timestamp - current_time) : 
                           (current_time - timestamp);
  if (abs_diff > max_drift_param) {
    const int32_t diff = (int32_t)(timestamp - current_time);
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Timestamp drift too large: %d seconds (max: %u)", diff, max_drift_param);
#endif
    return false;
  }
  
  return true;
}

uint32_t AutoTimeSync::calculateConsensusTime() const {
  if (sample_count <= 0) return 0;
  
  const uint32_t current_time = _rtc->getCurrentTime();
  
  uint32_t time_estimates[MAX_SAMPLES];
  int count = 0;
  
  // Build array of current time estimates from samples
  for (int i = 0; i < sample_count && count < MAX_SAMPLES; i++) {
    if (!isValidSampleIndex(i)) continue;
    
    if (isRecentSample(samples[i].received_at, current_time)) {
      uint32_t age = 0;
      if (current_time >= samples[i].received_at) {
        age = current_time - samples[i].received_at;
      }
      
      uint32_t estimate = samples[i].timestamp;
      if (age > 0 && estimate <= (UINT32_MAX - age)) {
        estimate += age;
      }
      
      time_estimates[count++] = estimate;
      
#if MESH_DEBUG
      MESH_DEBUG_PRINTLN("AutoTimeSync: Sample %d - original: %u, age: %us, estimate: %u", 
                         i, samples[i].timestamp, age, estimate);
#endif
    }
  }
  
  // Fall back to all samples if none are recent
  if (count == 0) {
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: No recent samples, using all samples");
#endif
    for (int i = 0; i < sample_count && count < MAX_SAMPLES; i++) {
      if (isValidSampleIndex(i)) {
        uint32_t age = 0;
        if (current_time >= samples[i].received_at) {
          age = current_time - samples[i].received_at;
        }
        
        uint32_t estimate = samples[i].timestamp;
        if (age > 0 && estimate <= (UINT32_MAX - age)) {
          estimate += age;
        }
        
        time_estimates[count++] = estimate;
      }
    }
  }
  
  if (count == 0) return 0;
  
  // Sort and find median
  insertionSort(time_estimates, count);
  
  // Calculate median safely
  uint32_t median;
  if (count % 2 == 0) {
    const uint32_t mid1 = time_estimates[count / 2 - 1];
    const uint32_t mid2 = time_estimates[count / 2];
    median = (mid1 >> 1) + (mid2 >> 1) + ((mid1 & 1) + (mid2 & 1)) / 2;
  } else {
    median = time_estimates[count / 2];
  }
  
#if MESH_DEBUG
  MESH_DEBUG_PRINTLN("AutoTimeSync: Consensus from %d samples: %u", count, median);
#endif
  
  return median;
}

void AutoTimeSync::addSample(uint32_t timestamp, uint8_t hop_count, uint8_t sender_hash) {
  if (!_rtc) return;
  
  const uint32_t current_time = _rtc->getCurrentTime();
  
  // Look for existing sample from same sender
  const int existing_idx = findExistingSample(sender_hash);
  
  if (existing_idx >= 0 && existing_idx < MAX_SAMPLES && existing_idx < sample_count) {
    // Update existing sample
    samples[existing_idx].timestamp = timestamp;
    samples[existing_idx].received_at = current_time;
    samples[existing_idx].hop_count = hop_count;
    
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Updated sample from %02X, hops=%d, time=%u",
                       sender_hash, hop_count, timestamp);
#endif
    return;
  }
  
  // Add new sample
  if (sample_count < MAX_SAMPLES) {
    samples[sample_count].timestamp = timestamp;
    samples[sample_count].received_at = current_time;
    samples[sample_count].hop_count = hop_count;
    samples[sample_count].pub_key_hash = sender_hash;
    sample_count++;
    
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Added sample from %02X, hops=%d, time=%u (%d total)",
                       sender_hash, hop_count, timestamp, sample_count);
#endif
  } else {
    // Buffer full - replace oldest
    int oldest_idx = 0;
    uint32_t oldest_time = samples[0].received_at;
    
    for (int i = 1; i < MAX_SAMPLES; i++) {
      if (samples[i].received_at < oldest_time) {
        oldest_time = samples[i].received_at;
        oldest_idx = i;
      }
    }
    
    samples[oldest_idx].timestamp = timestamp;
    samples[oldest_idx].received_at = current_time;
    samples[oldest_idx].hop_count = hop_count;
    samples[oldest_idx].pub_key_hash = sender_hash;
    
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Replaced oldest sample from %02X, hops=%d, time=%u (buffer full)",
                       sender_hash, hop_count, timestamp);
#endif
  }
}

uint32_t AutoTimeSync::calculateTransmissionDelay(uint8_t hop_count, uint32_t airtime_ms) {
  const uint32_t processing_delay_per_hop = 75;   // Processing delay per hop
  const uint32_t average_backoff = 200;           // Average backoff delay
  
  uint32_t total_delay = airtime_ms;
  
  // Cap hop count to prevent overflow
  const uint8_t max_calc_hops = (hop_count > 8) ? 8 : hop_count;
  
  for (uint8_t i = 0; i < max_calc_hops; i++) {
    // Check for overflow
    if (total_delay > UINT32_MAX - processing_delay_per_hop - average_backoff) {
      return UINT32_MAX;
    }
    total_delay += processing_delay_per_hop + average_backoff;
  }
  
  return total_delay;
}

int AutoTimeSync::getRecentSampleCount() const {
  if (!_rtc || sample_count <= 0) return 0;
  
  const uint32_t current_time = _rtc->getCurrentTime();
  int recent_count = 0;
  
  for (int i = 0; i < sample_count; i++) {
    if (isValidSampleIndex(i) && isRecentSample(samples[i].received_at, current_time)) {
      recent_count++;
    }
  }
  
  return recent_count;
}

bool AutoTimeSync::isRecentSample(uint32_t received_at, uint32_t current_time) const {
  if (current_time >= received_at) {
    return (current_time - received_at) <= RECENT_WINDOW_SECS;
  } else {
    return false;  // Future timestamp or wraparound
  }
}

uint32_t AutoTimeSync::calculateTolerance(uint32_t sample_age) const {
  // Base tolerance plus 1 second per minute of age
  const uint32_t age_minutes = sample_age / 60;
  const uint32_t tolerance = BASE_TOLERANCE_SECS + age_minutes;
  
  // Cap at 10 minutes
  return (tolerance > 600) ? 600 : tolerance;
}

bool AutoTimeSync::isValidSampleIndex(int index) const {
  return (index >= 0 && index < sample_count && index < MAX_SAMPLES);
}

int AutoTimeSync::getUniqueSources() const {
  if (sample_count <= 0) return 0;
  
  int unique_count = 0;
  
  for (int i = 0; i < sample_count; i++) {
    if (!isValidSampleIndex(i)) continue;
    
    const uint8_t hash = samples[i].pub_key_hash;
    bool already_counted = false;
    
    // Check if hash already counted
    for (int j = 0; j < i; j++) {
      if (isValidSampleIndex(j) && samples[j].pub_key_hash == hash) {
        already_counted = true;
        break;
      }
    }
    
    if (!already_counted) {
      unique_count++;
    }
  }
  
  return unique_count;
}

void AutoTimeSync::insertionSort(uint32_t arr[], int n) {
  for (int i = 1; i < n; i++) {
    uint32_t key = arr[i];
    int j = i - 1;
    
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
}

int AutoTimeSync::findExistingSample(uint8_t sender_hash) const {
  for (int i = 0; i < sample_count; i++) {
    if (isValidSampleIndex(i) && samples[i].pub_key_hash == sender_hash) {
      return i;
    }
  }
  return -1;
}

TimeReq AutoTimeSync::generateTimeRequest(uint8_t pool_size) {
  if (!_rtc) {
    TimeReq req = {0};
    return req;
  }
  
  // Generate a new nonce
  last_nonce = _rtc->getCurrentTime() + (rand() & 0xFFFF);
  
  // Use provided pool size or default to current peer count
  uint8_t effective_pool_size = (pool_size > 0) ? pool_size : time_req_pool_size;
  if (effective_pool_size == 0) {
    effective_pool_size = 8;  // Default fallback
  }
  
  TimeReq req;
  req.magic = TIME_REQ_MAGIC;
  req.nonce = last_nonce;
  req.pool_size = effective_pool_size;
  
#if MESH_DEBUG
  MESH_DEBUG_PRINTLN("AutoTimeSync: Generated time request, nonce=%u, pool_size=%u", last_nonce, effective_pool_size);
#endif
  
  return req;
}

bool AutoTimeSync::processTimeRequest(const TimeReq& req, uint8_t sender_hash) {
  if (!_rtc || !enabled) return false;
  
  // Validate magic number
  if (req.magic != TIME_REQ_MAGIC) {
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Invalid time request magic: %08X", req.magic);
#endif
    return false;
  }
  
  // Validate pool size
  if (req.pool_size == 0 || req.pool_size > 64) {
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Invalid pool size: %u", req.pool_size);
#endif
    return false;
  }
  
  // Random selection: respond with probability pool_size/3 / pool_size = 1/3
  uint32_t random_val = rand() % req.pool_size;
  bool should_respond = (random_val < (req.pool_size / 3));
  
#if MESH_DEBUG
  MESH_DEBUG_PRINTLN("AutoTimeSync: Time request from %02X, nonce=%u, pool_size=%u, respond=%s", 
                     sender_hash, req.nonce, req.pool_size, should_respond ? "YES" : "NO");
#endif
  
  return should_respond;
}

bool AutoTimeSync::processTimeReply(const TimeReply& reply, uint8_t sender_hash) {
  if (!_rtc || !enabled) return false;
  
  // Validate magic number
  if (reply.magic != TIME_REPLY_MAGIC) {
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Invalid time reply magic: %08X", reply.magic);
#endif
    return false;
  }
  
  // Validate nonce matches our last request
  if (reply.nonce != last_nonce) {
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Time reply nonce mismatch: got %u, expected %u", reply.nonce, last_nonce);
#endif
    return false;
  }
  
  // Verify MAC
  if (!verifyReplyMAC(reply, sender_hash)) {
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Time reply MAC verification failed");
#endif
    return false;
  }
  
  // Process the timestamp (similar to existing processSample)
  uint32_t current_time = _rtc->getCurrentTime();
  
  // Add some estimated delay for processing
  uint32_t estimated_delay_ms = calculateTransmissionDelay(reply.hop_count, 50);  // Assume 50ms airtime
  
  // Process as a normal sample
  processSample(reply.timestamp, reply.hop_count, sender_hash, estimated_delay_ms);
  
#if MESH_DEBUG
  MESH_DEBUG_PRINTLN("AutoTimeSync: Processed time reply from %02X, timestamp=%u, hops=%u", 
                     sender_hash, reply.timestamp, reply.hop_count);
#endif
  
  return true;
}

bool AutoTimeSync::verifyReplyMAC(const TimeReply& reply, uint8_t sender_hash) const {
  uint8_t expected_mac = calculateReplyMAC(reply.nonce, reply.timestamp, sender_hash);
  return (reply.mac == expected_mac);
}

uint8_t AutoTimeSync::calculateReplyMAC(uint32_t nonce, uint32_t timestamp, uint8_t sender_hash) const {
  // Simple MAC calculation using sender hash as key
  // In production, this would use a proper shared secret
  uint8_t data[8];
  memcpy(&data[0], &nonce, 4);
  memcpy(&data[4], &timestamp, 4);
  
  uint8_t mac = sender_hash;
  for (int i = 0; i < 8; i++) {
    mac ^= data[i];
    mac = (mac << 1) | (mac >> 7);  // Rotate left
  }
  
  return mac;
}

bool AutoTimeSync::performPlausibilityCheck(uint32_t candidate_time, uint32_t rtc_now) const {
  if (last_sync_rtc == 0) return true;  // No prior sync data
  
  // Calculate time elapsed since last sync
  uint32_t dt = 0;
  if (rtc_now >= last_sync_rtc) {
    dt = rtc_now - last_sync_rtc;
  } else {
    // Handle RTC rollover (unlikely but possible)
    dt = (UINT32_MAX - last_sync_rtc) + rtc_now + 1;
  }
  
  // Predict expected time based on drift rate
  float predicted_offset = drift_rate * (float)dt;
  uint32_t predicted_time = rtc_now + (int32_t)predicted_offset;
  
  // Check if candidate is within tolerance
  float deviation = fabsf((float)((int32_t)(candidate_time - predicted_time)));
  
  if (deviation > drift_tolerance) {
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Plausibility check failed: candidate=%u, predicted=%u, deviation=%.1f > %.1f", 
                       candidate_time, predicted_time, deviation, drift_tolerance);
#endif
    return false;
  }
  
#if MESH_DEBUG
  MESH_DEBUG_PRINTLN("AutoTimeSync: Plausibility check passed: deviation=%.1f <= %.1f", deviation, drift_tolerance);
#endif
  return true;
}

void AutoTimeSync::updateDriftRate(uint32_t consensus_time, uint32_t rtc_now) {
  // Skip drift calculation for initial sync
  if (last_sync_rtc == 0) {
    last_sync_rtc = rtc_now;
    return;
  }
  
  // Calculate time elapsed since last sync
  uint32_t dt = 0;
  if (rtc_now >= last_sync_rtc) {
    dt = rtc_now - last_sync_rtc;
  } else {
    // Handle RTC rollover (unlikely but possible)
    dt = (UINT32_MAX - last_sync_rtc) + rtc_now + 1;
  }
  
  // Avoid division by zero or very small time intervals
  if (dt < 60) {  // Minimum 1 minute between measurements
    return;
  }
  
  // Calculate offset and measured drift rate
  int32_t offset = (int32_t)(consensus_time - rtc_now);
  float measured_rate = (float)offset / (float)dt;
  
  // Apply exponential smoothing
  drift_rate = drift_rate * (1.0f - drift_alpha) + measured_rate * drift_alpha;
  
  // Clamp to maximum drift rate (use configured max drift rate)
  if (drift_rate > max_drift_rate) {
    drift_rate = max_drift_rate;
  } else if (drift_rate < -max_drift_rate) {
    drift_rate = -max_drift_rate;
  }
  
  // Update last sync RTC time
  last_sync_rtc = rtc_now;
  
#if MESH_DEBUG
  MESH_DEBUG_PRINTLN("AutoTimeSync: Drift rate updated: offset=%d, dt=%u, measured_rate=%.6f, new_drift_rate=%.6f", 
                     offset, dt, measured_rate, drift_rate);
#endif
}

} 