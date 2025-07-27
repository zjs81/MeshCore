// NRFSleep.cpp
#include "NRFSleep.h"

#ifdef NRF52_PLATFORM

#include <Mesh.h>
#include <Dispatcher.h>
#include <nrf_rtc.h>
#include <helpers/SimpleHardwareTimer.h>
#include <cmath>  // For fabsf() float comparisons

#ifdef MAX_CONTACTS
  #include <bluefruit.h>
#endif

// Define button pressed state if not already defined by platformio.ini
#ifndef USER_BTN_PRESSED
#define USER_BTN_PRESSED LOW  // Default for most boards
#endif

// -----------------------------------------------------------------------------
// Types & Constants
// -----------------------------------------------------------------------------
enum WakeReason {
  NONE,
  RADIO,
  BUSY,
  BUTTON,
  TIMER,
  BLE
};

// Exponential back‑off parameters
static constexpr uint32_t BASE_SLEEP_MS = 10000;   // 10 s
static constexpr uint32_t MAX_SLEEP_MS  = 90000;   // 1.5 min

// -----------------------------------------------------------------------------
// Static state
// -----------------------------------------------------------------------------
static volatile WakeReason  wakeReason        = NONE;
static uint32_t             lastSleepCheckMs  = 0;
static mesh::Radio*         radioInstance     = nullptr;
static mesh::Dispatcher*    dispatcherInstance = nullptr;
static uint32_t             backoffSleepMs    = BASE_SLEEP_MS;
static uint32_t             lastRadioWakeMs   = 0;
static uint32_t             lastBLEActivityMs = 0;  // Track BLE connection attempts

// Duty cycle parameters for radio power management
static bool                 dutyCycleEnabled  = false;
static uint32_t             rxWindowMs        = 150;    // RX window duration - increased for better reception
static uint32_t             dutyCycleIntervalMs = 1000; // Total cycle time - reduced for more frequent checks
static uint32_t             lastRxWindowMs    = 0;      // When we last started RX window
static bool                 radioInRxMode     = false;  // Track radio state

// Adaptive duty cycle parameters with circular buffer for memory safety
static bool                 adaptiveDutyCycle = false;
static float                minDutyCyclePercent = 70.0f; // Minimum duty cycle for reliable packet reception
static float                maxDutyCyclePercent = 95.0f; // Maximum duty cycle for high-activity periods  
static float                currentDutyCyclePercent = 75.0f; // Current adaptive duty cycle - start higher
static uint32_t             lastAdaptiveUpdateMs = 0;   // When we last updated adaptive settings
static uint32_t             lastPacketActivityMs = 0;  // When we last saw any packet activity

// Circular buffer for learning history (fixed memory, never overflows)
#define LEARNING_BUFFER_SIZE 24  // 24 hours of history (1 sample per hour)
struct LearningWindow {
  uint16_t totalWindows;    // Total RX windows in this hour
  uint16_t activeWindows;   // RX windows with activity in this hour
  uint8_t  dutyCyclePercent; // Duty cycle used during this hour
  uint8_t  reserved;        // Padding for alignment
};

static LearningWindow       learningBuffer[LEARNING_BUFFER_SIZE];
static uint8_t              bufferHead = 0;            // Current position in circular buffer
static uint8_t              bufferCount = 0;           // Number of valid entries (0-24)
static uint32_t             currentHourWindows = 0;    // Windows in current hour
static uint32_t             currentHourActive = 0;     // Active windows in current hour
static uint32_t             lastHourUpdateMs = 0;      // When we last rotated the hour
static bool                 currentWindowHasActivity = false; // Track activity in current RX window

// -----------------------------------------------------------------------------
// Interrupt Service Routines
// -----------------------------------------------------------------------------
static void ISR_onDIO1()   { wakeReason = RADIO; }
static void ISR_onBusy()   { wakeReason = BUSY;  }
static void ISR_onButton() { wakeReason = BUTTON; }

// -----------------------------------------------------------------------------
// Forward declarations for circular buffer functions
// -----------------------------------------------------------------------------
static void initLearningBuffer();
static void addWindowToCurrentHour(bool hasActivity);
static void rotateToNextHour();
static void updateAdaptiveDutyCycle();
static void manageRadioDutyCycle();
static float calculateAverageActivityRate();
static float calculateRecentActivityRate();
static void printLearningBufferStatus();

// Enhanced BLE monitoring functions
static void enableEnhancedBLEMonitoring();

// Static flag to prevent duplicate initialization
static bool nrfSleepInitialized = false;

// -----------------------------------------------------------------------------
// Enhanced BLE Activity Detection
// -----------------------------------------------------------------------------
#ifdef MAX_CONTACTS

// Flag to track if enhanced BLE monitoring is enabled
static bool enhanced_ble_monitoring = false;

static void enableEnhancedBLEMonitoring() {
#ifdef MAX_CONTACTS
  if (!enhanced_ble_monitoring) {
    enhanced_ble_monitoring = true;
    Serial.println("DEBUG: Enhanced BLE monitoring enabled for early connection detection");
  }
#endif
}

#endif // MAX_CONTACTS

// -----------------------------------------------------------------------------
// Common initialization helper
// -----------------------------------------------------------------------------
static void initializeInterruptsAndTimer() {
  if (nrfSleepInitialized) {
    Serial.println("DEBUG: NRFSleep already initialized, skipping redundant setup");
    return;
  }

  pinMode(LORA_DIO_1, INPUT);
  attachInterrupt(digitalPinToInterrupt(LORA_DIO_1), ISR_onDIO1, CHANGE);

  pinMode(LORA_BUSY, INPUT);
  attachInterrupt(digitalPinToInterrupt(LORA_BUSY), ISR_onBusy, RISING);

  #ifdef BUTTON_PIN
  pinMode(BUTTON_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), ISR_onButton, RISING);
  #endif

  SimpleHardwareTimer::init();
  wakeReason       = NONE;
  lastSleepCheckMs = millis();
  lastRadioWakeMs  = 0;
  backoffSleepMs   = BASE_SLEEP_MS;

  nrfSleepInitialized = true;

  Serial.println("DEBUG: NRFSleep core initialization completed");
  Serial.printf("DEBUG: Using LORA_DIO_1 pin %d, LORA_BUSY pin %d\n", LORA_DIO_1, LORA_BUSY);
  #ifdef BUTTON_PIN
  Serial.printf("DEBUG: Button pin %d configured for RISING edge (active HIGH)\n", BUTTON_PIN);
  #endif

#ifdef MAX_CONTACTS
  enableEnhancedBLEMonitoring();
#endif
}

// -----------------------------------------------------------------------------
// Initialization
// -----------------------------------------------------------------------------
void NRFSleep::init() {
  radioInstance = nullptr;
  dispatcherInstance = nullptr;

  initializeInterruptsAndTimer();
  
  Serial.println("DEBUG: NRFSleep initialized (no radio)");
}

void NRFSleep::init(mesh::Radio* radio) {
  radioInstance = radio;
  if (!dispatcherInstance) {
    dispatcherInstance = nullptr;
  }

  initializeInterruptsAndTimer();

  static bool advancedFeaturesInitialized = false;
  if (!advancedFeaturesInitialized && radioInstance) {
    initLearningBuffer();
    
    enableAdaptiveDutyCycle(70.0f, 95.0f);
    
    advancedFeaturesInitialized = true;
    Serial.println("DEBUG: NRFSleep initialized with radio + adaptive duty cycling (production default)");
  } else if (radioInstance) {
    Serial.println("DEBUG: NRFSleep radio instance updated");
  } else {
    Serial.println("DEBUG: NRFSleep initialized with radio instance");
  }
}

void NRFSleep::init(mesh::Radio* radio, mesh::Dispatcher* dispatcher) {
  radioInstance = radio;
  dispatcherInstance = dispatcher;

  initializeInterruptsAndTimer();

  static bool advancedFeaturesInitialized = false;
  if (!advancedFeaturesInitialized && radioInstance) {
    initLearningBuffer();
    enableAdaptiveDutyCycle(70.0f, 95.0f);
    
    advancedFeaturesInitialized = true;
    Serial.println("DEBUG: NRFSleep initialized with radio + dispatcher + adaptive duty cycling (production default)");
  } else {
    Serial.println("DEBUG: NRFSleep radio and dispatcher instances updated");
  }
}

// -----------------------------------------------------------------------------
// Set dispatcher reference after mesh initialization
// -----------------------------------------------------------------------------
void NRFSleep::setDispatcher(mesh::Dispatcher* dispatcher) {
  dispatcherInstance = dispatcher;
  Serial.println("DEBUG: NRFSleep dispatcher reference updated");
}

// -----------------------------------------------------------------------------
// BLE activity notification
// -----------------------------------------------------------------------------
void NRFSleep::notifyBLEActivity() {
  lastBLEActivityMs = millis();
  Serial.println("DEBUG: BLE activity detected - preventing sleep for 10s");
}

void NRFSleep::notifyBLEConnectionAttempt() {
  lastBLEActivityMs = millis();
  Serial.println("DEBUG: BLE connection attempt detected - preventing sleep for 10s");
  
  notifyBLEActivity();
}

// -----------------------------------------------------------------------------
// Radio duty cycle configuration for power savings
// -----------------------------------------------------------------------------
void NRFSleep::setDutyCycle(uint32_t rx_window_ms, uint32_t interval_ms) {
  if (interval_ms < rx_window_ms) {
    Serial.println("ERROR: Duty cycle interval must be >= RX window");
    return;
  }
  
  rxWindowMs = rx_window_ms;
  dutyCycleIntervalMs = interval_ms;
  lastRxWindowMs = 0;  // Reset timing
  
  float dutyCyclePercent = (float)rx_window_ms / interval_ms * 100.0f;
  
  // Only enable duty cycling if hardware supports it
  if (radioInstance && radioInstance->supportsHardwareDutyCycle()) {
    // Convert to microseconds for hardware timer
    uint32_t rxPeriodUs = rx_window_ms * 1000;
    uint32_t sleepPeriodUs = (interval_ms - rx_window_ms) * 1000;
    
    if (radioInstance->startReceiveDutyCycle(rxPeriodUs, sleepPeriodUs)) {
      dutyCycleEnabled = true;  // Only enable if hardware duty cycle succeeds
      Serial.printf("DEBUG: Hardware duty cycle enabled: %ums RX every %ums (%.1f%% duty cycle)\n", 
                    rx_window_ms, interval_ms, dutyCyclePercent);
      
      // Estimated power savings
      float powerSavings = (1.0f - (float)rx_window_ms / interval_ms) * 100.0f;
      Serial.printf("DEBUG: Hardware-controlled power savings: %.0f%% (from ~5mA to ~%.1fmA)\n", 
                    powerSavings, 5.0f * rx_window_ms / interval_ms + 0.1f);
      return;
    }
  }
  
  // Hardware duty cycling not available - do not enable software fallback
  dutyCycleEnabled = false;  // Explicitly disable duty cycling
  Serial.printf("WARNING: Hardware duty cycle not supported - duty cycling disabled (requested: %ums RX every %ums, %.1f%% duty cycle)\n", 
                rx_window_ms, interval_ms, dutyCyclePercent);
  Serial.println("INFO: Radio will remain in continuous RX mode for maximum responsiveness");
  
  // Ensure radio is in continuous RX mode
  if (radioInstance && !radioInstance->isInRecvMode()) {
    radioInstance->startRecv();
    radioInRxMode = true;
  }
}

void NRFSleep::disableDutyCycle() {
  dutyCycleEnabled = false;
  adaptiveDutyCycle = false;
  Serial.println("DEBUG: Radio duty cycle disabled - continuous RX mode");
  
  // Ensure radio is in RX mode
  if (radioInstance && !radioInstance->isInRecvMode()) {
    radioInstance->startRecv();
    radioInRxMode = true;
  }
}

void NRFSleep::enableAdaptiveDutyCycle(float min_duty_percent, float max_duty_percent) {
  if (min_duty_percent < 1.0f || max_duty_percent > 95.0f || min_duty_percent >= max_duty_percent) {
    Serial.println("ERROR: Invalid adaptive duty cycle range");
    return;
  }
  
  // Check if hardware supports duty cycling before enabling
  if (!radioInstance || !radioInstance->supportsHardwareDutyCycle()) {
    Serial.println("WARNING: Adaptive duty cycle requires hardware duty cycling support - feature disabled");
    Serial.println("INFO: Radio will remain in continuous RX mode for maximum responsiveness");
    dutyCycleEnabled = false;
    adaptiveDutyCycle = false;
    
    // Ensure radio is in continuous RX mode
    if (radioInstance && !radioInstance->isInRecvMode()) {
      radioInstance->startRecv();
      radioInRxMode = true;
    }
    return;
  }
  
  minDutyCyclePercent = min_duty_percent;
  maxDutyCyclePercent = max_duty_percent;
  currentDutyCyclePercent = (min_duty_percent + max_duty_percent) / 2.0f; // Start in middle
  
  // Calculate initial interval based on fixed RX window and adaptive duty cycle
  dutyCycleIntervalMs = uint32_t(float(rxWindowMs) * 100.0f / currentDutyCyclePercent + 0.5f);
  
  dutyCycleEnabled = true;
  adaptiveDutyCycle = true;
  
  initLearningBuffer();  // Initialize learning system
  
  Serial.printf("DEBUG: Adaptive duty cycle enabled: %.1f%% to %.1f%%, starting at %.1f%%\n", 
                min_duty_percent, max_duty_percent, currentDutyCyclePercent);
  Serial.printf("DEBUG: Initial settings: %ums RX every %ums\n", rxWindowMs, dutyCycleIntervalMs);
  
  // Apply initial settings to hardware
  uint32_t rxPeriodUs = rxWindowMs * 1000;
  uint32_t sleepPeriodUs = (dutyCycleIntervalMs - rxWindowMs) * 1000;
  if (radioInstance->startReceiveDutyCycle(rxPeriodUs, sleepPeriodUs)) {
    Serial.println("DEBUG: Applied adaptive settings to hardware duty cycling");
  } else {
    Serial.println("ERROR: Failed to apply adaptive settings to hardware - disabling adaptive duty cycle");
    dutyCycleEnabled = false;
    adaptiveDutyCycle = false;
  }
}

// -----------------------------------------------------------------------------
// Adaptive duty cycle management
// -----------------------------------------------------------------------------
void NRFSleep::updateAdaptiveDutyCycle() {
  if (!adaptiveDutyCycle) return;

  uint32_t now = millis();
  
  // Only update every 5 minutes minimum between updates
  if (now - lastAdaptiveUpdateMs < 300000) {  // 5 minutes minimum between updates
    return;
  }
  lastAdaptiveUpdateMs = now;

  // Calculate activity rates
  float avgActivity = calculateAverageActivityRate();
  float recentActivity = calculateRecentActivityRate();
  
  // Handle early startup period gracefully - use recent activity as baseline if no history
  if (avgActivity < 0 && recentActivity >= 0) {
    // New deployment - use recent activity as initial baseline
    avgActivity = recentActivity;
    Serial.printf("DEBUG: Early startup - using recent activity %.2f%% as baseline\n", recentActivity * 100);
  } else if (avgActivity < 0 && recentActivity < 0) {
    // No data at all yet - use conservative defaults
    Serial.println("DEBUG: No activity data yet - maintaining current duty cycle");
    return;
  } else if (recentActivity < 0) {
    // No recent activity but have historical data - assume quiet period
    recentActivity = 0.0f;
    Serial.println("DEBUG: No recent activity - assuming quiet period");
  }

  Serial.printf("DEBUG: Adaptive duty cycle analysis: avg=%.2f%%, recent=%.2f%%, current=%.1f%%\n", 
                avgActivity * 100, recentActivity * 100, currentDutyCyclePercent);

  float oldDutyCycle = currentDutyCyclePercent;

  // Adjust duty cycle based on recent vs historical activity with conservative scaling
  if (recentActivity > avgActivity * 2.0f) {
    // High activity spike - increase duty cycle moderately (was 2.0x, now 1.5x)
    currentDutyCyclePercent = min(currentDutyCyclePercent * 1.5f, maxDutyCyclePercent);
  } else if (recentActivity > avgActivity * 1.5f) {
    // Moderate activity increase (was 1.5x, now 1.3x)
    currentDutyCyclePercent = min(currentDutyCyclePercent * 1.3f, maxDutyCyclePercent);
  } else if (recentActivity > avgActivity * 1.2f) {
    // Slight activity increase (was 1.2x, now 1.15x)
    currentDutyCyclePercent = min(currentDutyCyclePercent * 1.15f, maxDutyCyclePercent);
  } else if (recentActivity > avgActivity * 1.1f) {
    // Very slight increase (was 1.1x, now 1.05x)
    currentDutyCyclePercent = min(currentDutyCyclePercent * 1.05f, maxDutyCyclePercent);
  } else if (recentActivity < avgActivity * 0.5f) {
    // Activity has decreased significantly - reduce duty cycle conservatively
    float decreaseFactor = 0.9f;  // Conservative decrease (was 0.8f)
    if (recentActivity < avgActivity * 0.1f) {
      decreaseFactor = 0.8f;  // Still conservative (was 0.5f)
    }
    
    float minSafetyDutyCycle = max(70.0f, minDutyCyclePercent);
    currentDutyCyclePercent = max(currentDutyCyclePercent * decreaseFactor, minSafetyDutyCycle);
  }

  // Apply safety limits with improved bounds checking
  static constexpr float HARD_MINIMUM_DUTY_CYCLE = 10.0f; // Absolute minimum for reliability
  
  float effectiveMinimum = max(70.0f, max(minDutyCyclePercent, HARD_MINIMUM_DUTY_CYCLE));
  
  // Clamp to safe minimum
  if (currentDutyCyclePercent < effectiveMinimum) {
    currentDutyCyclePercent = effectiveMinimum;
    Serial.printf("DEBUG: Duty cycle clamped to %.1f%% minimum (safety: %.1f%%, user: %.1f%%)\n", 
                  effectiveMinimum, HARD_MINIMUM_DUTY_CYCLE, minDutyCyclePercent);
  }
  
  // Clamp to maximum
  if (currentDutyCyclePercent > maxDutyCyclePercent) {
    currentDutyCyclePercent = maxDutyCyclePercent;
    Serial.printf("DEBUG: Duty cycle clamped to %.1f%% maximum\n", maxDutyCyclePercent);
  }

  // Apply changes if significant (reduced threshold for more responsive adaptation)
  if (fabsf(currentDutyCyclePercent - oldDutyCycle) > 0.3f) {  // Was 0.5f, now 0.3f
    dutyCycleIntervalMs = uint32_t(float(rxWindowMs) * 100.0f / currentDutyCyclePercent + 0.5f);
    
    // Sanity check the calculated interval
    if (dutyCycleIntervalMs < rxWindowMs || dutyCycleIntervalMs > 60000) {
      Serial.printf("ERROR: Invalid duty cycle interval %ums calculated, reverting to safe defaults\n", dutyCycleIntervalMs);
      currentDutyCyclePercent = 20.0f;  // Safe default
      dutyCycleIntervalMs = uint32_t(float(rxWindowMs) * 100.0f / 20.0f + 0.5f);
    }
    
    Serial.printf("DEBUG: Adaptive duty cycle updated: activity=%.1f%% -> %.1f%% duty, %ums interval\n", 
                  recentActivity * 100, currentDutyCyclePercent, dutyCycleIntervalMs);
                  
    // Apply to hardware duty cycling if available
    if (radioInstance && radioInstance->supportsHardwareDutyCycle() && dutyCycleEnabled) {
      uint32_t rxPeriodUs = rxWindowMs * 1000;
      uint32_t sleepPeriodUs = (dutyCycleIntervalMs - rxWindowMs) * 1000;
      
      if (radioInstance->startReceiveDutyCycle(rxPeriodUs, sleepPeriodUs)) {
        Serial.println("DEBUG: Hardware duty cycle updated with new adaptive settings");
      } else {
        Serial.println("ERROR: Failed to update hardware duty cycle - disabling adaptive mode");
        adaptiveDutyCycle = false;
        dutyCycleEnabled = false;
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Circular buffer management for memory-safe learning
// -----------------------------------------------------------------------------
static void initLearningBuffer() {
  memset(learningBuffer, 0, sizeof(learningBuffer));
  bufferHead = 0;
  bufferCount = 0;
  currentHourWindows = 0;
  currentHourActive = 0;
  lastHourUpdateMs = millis();
  lastPacketActivityMs = millis();
  Serial.println("DEBUG: Learning buffer initialized (24-hour circular buffer)");
}

static void addWindowToCurrentHour(bool hasActivity) {
  currentHourWindows++;
  if (hasActivity) {
    currentHourActive++;
  }
  
  // Prevent overflow (extremely unlikely but safety first)
  if (currentHourWindows > 65000) {
    currentHourWindows = 65000;
  }
  if (currentHourActive > 65000) {
    currentHourActive = 65000;  
  }
}

static void markCurrentWindowActive() {
  if (!adaptiveDutyCycle) return;
  
  if (!currentWindowHasActivity) {
    currentWindowHasActivity = true;
    currentHourActive++;
    lastPacketActivityMs = millis();
    
    // Log activity detection for debugging
    Serial.println("DEBUG: Activity detected in current RX window");
  }
}

static void rotateToNextHour() {
  // Store current hour data in circular buffer
  learningBuffer[bufferHead].totalWindows = min(currentHourWindows, 65535);
  learningBuffer[bufferHead].activeWindows = min(currentHourActive, 65535);
  learningBuffer[bufferHead].dutyCyclePercent = (uint8_t)min(currentDutyCyclePercent, 255.0f);
  learningBuffer[bufferHead].reserved = 0;
  
  Serial.printf("DEBUG: Hour complete - %d/%d active windows (%.1f%% activity) at %.1f%% duty cycle [Buffer pos %d]\n",
                currentHourActive, currentHourWindows, 
                currentHourWindows > 0 ? (float)currentHourActive / currentHourWindows * 100.0f : 0.0f,
                currentDutyCyclePercent, bufferHead);
  
  // Print buffer status after rotation
  printLearningBufferStatus();
  
  // Move to next buffer position (circular)
  bufferHead = (bufferHead + 1) % LEARNING_BUFFER_SIZE;
  if (bufferCount < LEARNING_BUFFER_SIZE) {
    bufferCount++;
  }
  
  // Reset current hour counters
  currentHourWindows = 0;
  currentHourActive = 0;
  lastHourUpdateMs = millis();
}

// Helper functions for adaptive duty cycling
static float calculateAverageActivityRate() {
  if (bufferCount == 0) return -1.0f;  // No data
  
  uint32_t totalWindowsSum = 0;
  uint32_t activeWindowsSum = 0;
  
  for (int i = 0; i < bufferCount; i++) {
    totalWindowsSum += learningBuffer[i].totalWindows;
    activeWindowsSum += learningBuffer[i].activeWindows;
  }
  
  if (totalWindowsSum == 0) return 0.0f;
  return (float)activeWindowsSum / totalWindowsSum;
}

static float calculateRecentActivityRate() {
  // Use current hour data for recent activity
  if (currentHourWindows == 0) return -1.0f;  // No recent data
  return (float)currentHourActive / currentHourWindows;
}

static void printLearningBufferStatus() {
  if (bufferCount == 0) return;
  
  Serial.printf("DEBUG: Learning buffer status (%d hours):\n", bufferCount);
  for (int i = 0; i < min(bufferCount, 5); i++) {  // Show last 5 hours
    int idx = (bufferHead - 1 - i + LEARNING_BUFFER_SIZE) % LEARNING_BUFFER_SIZE;
    if (idx < 0) idx += LEARNING_BUFFER_SIZE;
    
    float activity = learningBuffer[idx].totalWindows > 0 
      ? (float)learningBuffer[idx].activeWindows / learningBuffer[idx].totalWindows * 100.0f
      : 0.0f;
    Serial.printf("  Hour -%d: %d/%d windows (%.1f%% activity, %.1f%% duty)\n", 
                  i+1, learningBuffer[idx].activeWindows, learningBuffer[idx].totalWindows,
                  activity, (float)learningBuffer[idx].dutyCyclePercent);
  }
}

// -----------------------------------------------------------------------------
// Radio duty cycle management
// -----------------------------------------------------------------------------
void NRFSleep::manageRadioDutyCycle() {
  if (!dutyCycleEnabled || !radioInstance) return;
  
  uint32_t now = millis();
  
  // Disable duty cycling when BLE is connected for maximum responsiveness
  #ifdef MAX_CONTACTS
  static bool wasBLEConnected = false;
  static bool preBLEDutyCycleState = false;  // Remember duty cycle state before BLE connection
  static bool preBLEAdaptiveState = false;   // Remember adaptive state before BLE connection
  bool bleConnected = Bluefruit.connected();
  
  if (bleConnected && !wasBLEConnected) {
    // BLE just connected - save current state and disable duty cycling completely
    preBLEDutyCycleState = dutyCycleEnabled;
    preBLEAdaptiveState = adaptiveDutyCycle;
    
    if (radioInstance->supportsHardwareDutyCycle() && dutyCycleEnabled) {
      radioInstance->idle();  // Stop hardware duty cycling
      dutyCycleEnabled = false;  // Disable duty cycling flag
      Serial.println("DEBUG: BLE connected - duty cycling disabled for maximum responsiveness");
    }
    
    // Ensure radio is in continuous receive mode
    radioInstance->startRecv();
    radioInRxMode = true;
    wasBLEConnected = true;
  } else if (!bleConnected && wasBLEConnected) {
    // BLE just disconnected - restore previous duty cycle state
    if (radioInstance->supportsHardwareDutyCycle() && preBLEDutyCycleState) {
      dutyCycleEnabled = preBLEDutyCycleState;
      adaptiveDutyCycle = preBLEAdaptiveState;
      
      // Restart duty cycling with current settings
      uint32_t rxPeriodUs = rxWindowMs * 1000;
      uint32_t sleepPeriodUs = (dutyCycleIntervalMs - rxWindowMs) * 1000;
      if (radioInstance->startReceiveDutyCycle(rxPeriodUs, sleepPeriodUs)) {
        Serial.printf("DEBUG: BLE disconnected - duty cycling restored (%.1f%% duty cycle)\n", 
                      currentDutyCyclePercent);
      }
    }
    wasBLEConnected = false;
  }
  
  // If BLE is connected, skip duty cycling management (duty cycle is now disabled)
  if (bleConnected) {
    return;
  }
  #endif
  
  // Check if we need to rotate to the next hour in our learning buffer
  if (adaptiveDutyCycle && (now - lastHourUpdateMs >= 3600000)) { // 1 hour = 3600000ms
    rotateToNextHour();
  }
  
  // Update adaptive parameters if enabled
  updateAdaptiveDutyCycle();
  
  // Hardware duty cycling handles timing automatically - just monitor for activity
  if (radioInstance->supportsHardwareDutyCycle()) {
    // Hardware is handling duty cycling - we just need to track activity for adaptive algorithm
    if (adaptiveDutyCycle) {
      // Check for preamble detection or packet activity for learning
      if (radioInstance->isPreambleDetected() || radioInstance->isReceiving()) {
        if (!currentWindowHasActivity) {
          currentWindowHasActivity = true;
          currentHourActive++;
          lastPacketActivityMs = now;
          Serial.println("DEBUG: Hardware duty cycle detected activity");
        }
      }
    }
    return;  // Hardware handles the rest
  }
  
  // No software duty cycling fallback - duty cycling is only enabled for hardware-supported radios
  // If we reach this point, it means duty cycling was somehow enabled without hardware support,
  // which should not happen with the updated configuration logic
  Serial.println("WARNING: Duty cycling enabled but hardware support not detected - this should not happen");
}

// -----------------------------------------------------------------------------
// Main sleep-management state machine
// -----------------------------------------------------------------------------
void NRFSleep::manageSleepLoop() {
  uint32_t now = millis();
  if (now - lastSleepCheckMs < 5000) return;  // Check every 5 seconds
  lastSleepCheckMs = now;

  // 0) Manage radio duty cycle for power savings
  manageRadioDutyCycle();

  // 1) Don't sleep if we just had LoRa activity or BUSY
  static uint32_t lastLoraMs = 0;
  if (wakeReason == RADIO || digitalRead(LORA_BUSY)) {
    lastLoraMs   = now;
    lastRadioWakeMs = now;  // Track when we last had radio activity
    wakeReason   = NONE;
    backoffSleepMs = BASE_SLEEP_MS;
    
    // Temporarily disable duty cycling during active periods
    if (dutyCycleEnabled) {
      Serial.println("DEBUG: Disabling duty cycle temporarily due to radio activity");
      if (radioInstance && !radioInstance->isInRecvMode()) {
        radioInstance->startRecv();
        radioInRxMode = true;
      }
    }
    return;
  }
  
  // 2) Don't sleep for 5 seconds after any radio activity to allow packet processing
  if (now - lastLoraMs < 5000) {
    // Keep radio active during packet processing period
    if (dutyCycleEnabled && radioInstance && !radioInstance->isInRecvMode()) {
      radioInstance->startRecv();
      radioInRxMode = true;
      Serial.println("DEBUG: Keeping radio active for packet processing");
    }
    return;
  }

  // 3) Check for pending packets (both inbound and outbound) if we have dispatcher access
  if (dispatcherInstance) {
    int outboundCount = dispatcherInstance->getOutboundCount(now);
    int inboundCount = dispatcherInstance->getInboundCount(now);
    if (outboundCount > 0 || inboundCount > 0) {
      Serial.printf("DEBUG: %d outbound, %d inbound packets pending - not sleeping\n", 
                    outboundCount, inboundCount);
      
      // Ensure radio is active for packet processing
      if (dutyCycleEnabled && radioInstance && !radioInstance->isInRecvMode()) {
        radioInstance->startRecv();
        radioInRxMode = true;
        Serial.println("DEBUG: Activating radio for pending packets");
      }
      return;
    }
  }

  // 4) Re-enable duty cycling after activity period ends
  if (dutyCycleEnabled && (now - lastLoraMs >= 5000)) {
    // Resume normal duty cycling (hardware only)
    if (radioInstance->supportsHardwareDutyCycle()) {
      // Hardware duty cycling resumes automatically - just log
      Serial.println("DEBUG: Resuming hardware duty cycle after activity period");
    }
    // Note: Software duty cycling is no longer supported
  }

  // 5) Pick sleep duration
  bool bleConnected = false;
  bool recentBLEActivity = false;
  bool recentRadioActivity = false;
  
  #ifdef MAX_CONTACTS
    bleConnected = Bluefruit.connected();
    // Check for recent BLE activity within the last 10 seconds
    recentBLEActivity = (now - lastBLEActivityMs < 10000);
    
    // Additional check: if we just connected, extend BLE activity period
    if (bleConnected && !recentBLEActivity) {
      Serial.println("DEBUG: Active BLE connection detected - extending wake period");
      lastBLEActivityMs = now;  // Reset timer to keep device awake
      recentBLEActivity = true;
    }
    
    // Enhanced monitoring: check if advertising is active (indicates potential connections)
    if (enhanced_ble_monitoring && !bleConnected && Bluefruit.Advertising.isRunning()) {
      // Device is advertising but not connected - check if we recently woke from radio activity
      // This could indicate a connection attempt in progress
      if ((now - lastRadioWakeMs < 2000) && !recentBLEActivity) {
        Serial.println("DEBUG: Advertising active with recent radio wake - potential BLE connection attempt");
        lastBLEActivityMs = now;  // Extend wake time
        recentBLEActivity = true;
      }
    }
  #endif
  
  // Check for recent radio activity within the last 30 seconds
  recentRadioActivity = (now - lastRadioWakeMs < 30000);

  uint32_t sleepMs;
  if (bleConnected || recentBLEActivity) {
    // BLE activity - use very short sleep for responsiveness
    sleepMs = 1000;
    backoffSleepMs = BASE_SLEEP_MS;
    wakeReason = BLE;
  } else if (recentRadioActivity) {
    // Recent radio activity - use shorter sleep than normal backoff
    sleepMs = 3000;  // 3 seconds instead of full backoff
    backoffSleepMs = BASE_SLEEP_MS;  // Reset backoff since we had activity
  } else {
    // No recent activity - use exponential backoff
    sleepMs = backoffSleepMs;
    backoffSleepMs = min(backoffSleepMs * 2, MAX_SLEEP_MS);
  }

  Serial.printf("DEBUG: Sleeping %ums (BLE=%d, RecentBLE=%d, RecentRadio=%d, DutyCycle=%d, HW=%d)\n", 
                sleepMs, bleConnected, recentBLEActivity, recentRadioActivity, dutyCycleEnabled, 
                radioInstance ? radioInstance->supportsHardwareDutyCycle() : 0);

  // 6) Configure radio for sleep period
  if (dutyCycleEnabled) {
    if (radioInstance && radioInstance->supportsHardwareDutyCycle()) {
      // Hardware duty cycling handles radio state automatically
      Serial.println("DEBUG: Hardware duty cycling active during sleep");
    }
    // Note: Software duty cycling is no longer supported
  } else {
    // Continuous mode - ensure radio is in receive mode
    if (radioInstance && !radioInstance->isInRecvMode()) {
      Serial.println("DEBUG: Starting radio RX mode before sleep");
      radioInstance->startRecv();
    }
  }

  // 7) Arm timer & clear flag with error checking
  if (!SimpleHardwareTimer::start(sleepMs)) {
    Serial.println("ERROR: Failed to start hardware timer - using delay fallback");
    delay(min(sleepMs, 1000UL));  // Fallback to delay with 1s max
    return;
  }
  wakeReason = NONE;

  // Turn off LEDs to save power during sleep
  
  // Primary LED control using board-specific definitions
  #ifdef LED_PIN
    #ifdef LED_STATE_ON
      digitalWrite(LED_PIN, !LED_STATE_ON);  // Turn off using inverted state
    #else
      digitalWrite(LED_PIN, LOW);  // Default assumption
    #endif
  #endif
  
  #ifdef LED_BUILTIN
    #ifdef LED_STATE_ON
      digitalWrite(LED_BUILTIN, !LED_STATE_ON);  // Turn off using inverted state
    #else
      digitalWrite(LED_BUILTIN, LOW);  // Default assumption
    #endif
  #endif
  
  #ifdef PIN_LED
    digitalWrite(PIN_LED, LOW);  // Usually active high
  #endif
  
  // RGB LEDs - only control if not already controlled by LED_PIN
  #if defined(LED_RED) && (!defined(LED_PIN) || (LED_PIN != LED_RED))
    #ifdef LED_STATE_ON
      digitalWrite(LED_RED, !LED_STATE_ON);
    #else
      digitalWrite(LED_RED, HIGH);  // Usually active low on RGB LEDs
    #endif
  #endif
  
  #if defined(LED_GREEN) && (!defined(LED_PIN) || (LED_PIN != LED_GREEN))
    #ifdef LED_STATE_ON
      digitalWrite(LED_GREEN, !LED_STATE_ON);
    #else
      digitalWrite(LED_GREEN, HIGH);  // Usually active low on RGB LEDs
    #endif
  #endif
  
  #if defined(LED_BLUE) && (!defined(LED_PIN) || (LED_PIN != LED_BLUE))
    #ifdef LED_STATE_ON
      digitalWrite(LED_BLUE, !LED_STATE_ON);
    #else
      digitalWrite(LED_BLUE, HIGH);  // Usually active low on RGB LEDs
    #endif
  #endif
  
  // LoRa TX LED (usually inverted)
  #ifdef P_LORA_TX_LED
    digitalWrite(P_LORA_TX_LED, HIGH);  // Usually inverted - HIGH for off
  #endif

  // 8) Sleep until timer expires *or* any wake source
  uint32_t sleepStartMs = millis();
  uint32_t maxSleepMs = sleepMs + 1000;  // Add 1s safety margin
  
  while (!SimpleHardwareTimer::isExpired()) {
    // Safety timeout check to prevent infinite loops
    uint32_t sleepElapsed = millis() - sleepStartMs;
    if (sleepElapsed > maxSleepMs) {
      Serial.printf("WARNING: Sleep timeout after %ums (expected %ums) - breaking loop\n", 
                    sleepElapsed, sleepMs);
      wakeReason = TIMER;
      break;
    }
    
    sd_app_evt_wait();
    
    // if our ISR fired...
    if (wakeReason != NONE)          break;
    // or if DIO1 lingers high...
    if (digitalRead(LORA_DIO_1))     { wakeReason = RADIO;  break; }
    // or if BUSY is asserted...
    if (digitalRead(LORA_BUSY))      { wakeReason = BUSY;   break; }
    // or if button pressed...
    #ifdef BUTTON_PIN
      if (digitalRead(BUTTON_PIN)==USER_BTN_PRESSED) { wakeReason = BUTTON; break; }
    #endif
  }
  SimpleHardwareTimer::stop();

  // 9) Figure out what actually woke us
  WakeReason reason = (wakeReason!=NONE ? wakeReason : TIMER);
  const char* what =
    reason==RADIO  ? "RADIO"  :
    reason==BUSY   ? "BUSY"   :
    reason==BUTTON ? "BUTTON" :
    reason==BLE    ? "BLE"    : "TIMER";
  Serial.printf("DEBUG: Woke: %s\n", what);

  // 10) Reset back‑off on real activity and track radio wakes
  if (reason==RADIO || reason==BUTTON || reason==BLE) {
    backoffSleepMs = BASE_SLEEP_MS;
    if (reason == RADIO) {
      lastRadioWakeMs = millis();  // Update radio wake time
    }
  }
}

#endif // NRF52_PLATFORM
