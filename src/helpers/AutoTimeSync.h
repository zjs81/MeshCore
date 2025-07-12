#pragma once

#include <Mesh.h>
#include <stdlib.h>

namespace mesh {

/**
 * Time synchronization for mesh nodes.
 * Collects timestamped samples and syncs when enough agree.
 */
struct TimeSample {
  uint32_t timestamp;      // Sender's timestamp
  uint32_t received_at;    // When we received this packet
  uint8_t hop_count;       // Hops this packet traveled
  uint8_t pub_key_hash;    // First byte of sender's public key
};

class AutoTimeSync {
private:
  // Configuration
  static const int MAX_SAMPLES = 8;
  static const uint32_t MIN_REASONABLE_TIME = 1577836800;   // 2020-01-01 UTC
  static const uint32_t MAX_REASONABLE_TIME = 2147483647;   // 2038-01-19 UTC  
  static const uint32_t RECENT_WINDOW_SECS = 25200;        // 7 hours
  static const uint32_t BASE_TOLERANCE_SECS = 30;          // Base consensus tolerance
  static const uint32_t MIN_SYNC_INTERVAL = 300;           // 5 minutes between syncs
  static const uint32_t MAX_INITIAL_DRIFT = 315360000;     // 10 years for initial sync
  static const uint32_t MAX_FUTURE_ALLOWANCE = 31536000;   // 1 year future tolerance
  
  // State
  TimeSample samples[MAX_SAMPLES];
  int sample_count;
  RTCClock* _rtc;
  
  // Settings
  bool enabled;
  uint8_t max_hops;
  uint8_t min_samples;
  uint32_t max_drift;
  uint32_t last_sync_time;
  uint32_t forwarded_samples_count;  // Count of forwarded samples processed
  
  // Internal functions
  bool isValidTimestamp(uint32_t timestamp, uint32_t current_time, uint32_t max_drift_param) const;
  uint32_t calculateConsensusTime() const;
  void addSample(uint32_t timestamp, uint8_t hop_count, uint8_t sender_hash);
  int getRecentSampleCount() const;
  bool isRecentSample(uint32_t received_at, uint32_t current_time) const;
  uint32_t calculateTolerance(uint32_t sample_age) const;
  bool isValidSampleIndex(int index) const;
  
  // Utilities
  static void insertionSort(uint32_t arr[], int n);
  int findExistingSample(uint8_t sender_hash) const;
  
public:
  explicit AutoTimeSync(RTCClock* rtc) 
    : _rtc(rtc), sample_count(0), last_sync_time(0), forwarded_samples_count(0) {
    if (!_rtc) {
      enabled = false;
      max_hops = 0;
      min_samples = 0;
      max_drift = 0;
      return;
    }
    
    // Default settings
    enabled = true;
    max_hops = 6;
    min_samples = 3;
    max_drift = 3600;
    
    memset(samples, 0, sizeof(samples));
  }
  
  // Configuration
  void configure(bool enable, uint8_t max_hop_count, uint8_t min_sample_count, uint32_t max_drift_secs) {
    if (!_rtc) {
      enabled = false;
      return;
    }
    
    enabled = enable;
    max_hops = (max_hop_count == 0 || max_hop_count > 8) ? 6 : max_hop_count;
    min_samples = (min_sample_count == 0 || min_sample_count > MAX_SAMPLES) ? 3 : min_sample_count;
    
    if (max_drift_secs == 0) {
      max_drift = 3600;
    } else if (max_drift_secs > MAX_INITIAL_DRIFT) {
      max_drift = MAX_INITIAL_DRIFT;
    } else {
      max_drift = max_drift_secs;
    }
  }
  
  // Process timestamp from direct packet
  void processSample(uint32_t sender_timestamp, uint8_t hop_count, uint8_t sender_hash, uint32_t estimated_delay_ms = 0);
  
  // Process timestamp from forwarded packet
  void processForwardedSample(uint32_t sender_timestamp, uint8_t hop_count, uint8_t sender_hash, uint32_t estimated_delay_ms = 0);
  
  // Estimate transmission delay
  static uint32_t calculateTransmissionDelay(uint8_t hop_count, uint32_t airtime_ms);
  
  // Check if ready to sync
  bool shouldSync() const;
  
  // Perform sync
  bool performSync();
  
  // Backwards compatible stats
  void getDetailedStats(int& total_samples, int& recent_samples, uint32_t& last_sync, bool& clock_is_set) const {
    uint32_t forwarded_count;  // Not used by old callers
    getDetailedStats(total_samples, recent_samples, last_sync, clock_is_set, forwarded_count);
  }
  
  // Full stats including forwarded count
  void getDetailedStats(int& total_samples, int& recent_samples, uint32_t& last_sync, bool& clock_is_set, uint32_t& forwarded_count) const {
    total_samples = (sample_count < 0) ? 0 : 
                   (sample_count > MAX_SAMPLES) ? MAX_SAMPLES : sample_count;
    recent_samples = getRecentSampleCount();
    last_sync = last_sync_time;
    clock_is_set = isClockSet();
    forwarded_count = forwarded_samples_count;
  }
  
  // Check if clock is set
  bool isClockSet() const {
    if (!_rtc) return false;
    uint32_t current_time = _rtc->getCurrentTime();
    return (current_time >= MIN_REASONABLE_TIME && current_time <= MAX_REASONABLE_TIME);
  }
  
  // Reset state for fresh sync
  void resetForResync() {
    last_sync_time = 0;
    sample_count = 0;
    forwarded_samples_count = 0;
    memset(samples, 0, sizeof(samples));
    
#if MESH_DEBUG
    MESH_DEBUG_PRINTLN("AutoTimeSync: Reset for fresh sync");
#endif
  }
  
  // Get number of unique sources
  int getUniqueSources() const;
};

} 