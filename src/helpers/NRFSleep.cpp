#include "NRFSleep.h"

// NRFSleep implementation is only available on NRF52 platforms
#ifdef NRF52_PLATFORM

#include <Mesh.h>
#include <nrf_rtc.h>

// Static member definitions
volatile uint32_t NRFSleep::last_lora_activity = 0;
uint32_t NRFSleep::last_sleep_check = 0;
mesh::Radio* NRFSleep::radio_instance = nullptr;
uint8_t NRFSleep::sleep_cycle_count = 0;

// LoRa DIO1 interrupt handler - captures TX/RX/Timeout events
void NRFSleep::lora_dio1_interrupt() {
  // Update activity timestamp (volatile access for interrupt safety)
  uint32_t current_time = millis();
  last_lora_activity = current_time;
  
  // Optional: Could add debug logging here if needed
  // Note: Keep interrupt handler minimal for best performance
}

// Configure RTC2 timer for wake-up from system off mode
void NRFSleep::setupRTCWake(uint32_t seconds) {
  // Use RTC2 (RTC0 is used by SoftDevice, RTC1 by Arduino)
  NRF_RTC2->TASKS_STOP = 1;
  NRF_RTC2->TASKS_CLEAR = 1;
  
  // Configure RTC2 for wake-up
  NRF_RTC2->PRESCALER = 4095; // 32768 Hz / (4095 + 1) = 8 Hz (125ms per tick)
  
  // Calculate compare value for desired seconds
  // 8 Hz = 8 ticks per second
  uint32_t compare_value = seconds * 8;
  NRF_RTC2->CC[0] = compare_value;
  
  // Enable RTC2 compare event and interrupt
  NRF_RTC2->INTENSET = RTC_INTENSET_COMPARE0_Msk;
  NRF_RTC2->EVTENSET = RTC_EVTENSET_COMPARE0_Msk;
  
  // Start RTC2
  NRF_RTC2->TASKS_START = 1;
  
  Serial.printf("DEBUG: RTC2 wake timer set for %d seconds (%d ticks)\n", seconds, compare_value);
}

void NRFSleep::init() {
  // Setup LoRa DIO1 interrupt for activity tracking
  pinMode(LORA_DIO_1, INPUT);
  
  // Initialize LoRa activity timestamp to allow immediate sleep on first boot
  last_lora_activity = 0;
  
  // Attach interrupt for LoRa events (TxDone, RxDone, RxTimeout, etc.)
  attachInterrupt(digitalPinToInterrupt(LORA_DIO_1), lora_dio1_interrupt, CHANGE);
  Serial.printf("DEBUG: LoRa DIO1 (pin %d) interrupt configured for CLIENT mode sleep\n", LORA_DIO_1);
  Serial.println("DEBUG: CLIENT mode - peripherals powered down only during deep sleep");
}

void NRFSleep::init(mesh::Radio* radio) {
  radio_instance = radio;
  init(); // Call the base init
  Serial.println("DEBUG: NRFSleep initialized with radio instance for airtime calculations");
}

bool NRFSleep::shouldDelayForLora() {
  uint32_t now = millis();
  
  // Calculate dynamic LoRa delay based on radio parameters
  uint32_t lora_delay_ms = getLoRaSleepDelayMs();
  
  // Check if LoRa activity occurred within the calculated delay period
  // Handle millis() overflow safely (though rare - occurs every ~49 days)
  uint32_t time_since_lora = (last_lora_activity == 0) ? lora_delay_ms + 1 : (now - last_lora_activity);
  bool lora_activity_recent = time_since_lora < lora_delay_ms;
  
  // Smart LoRa BUSY check: Only check if we haven't had very recent activity
  // This prevents interference with normal radio driver operations
  bool lora_busy = false;
  if (time_since_lora > 500) {  // Only check BUSY if no activity for 500ms
    lora_busy = digitalRead(LORA_BUSY) == HIGH;
  }
  
  if (lora_activity_recent || lora_busy) {
    const char* reason = lora_busy ? "LoRa BUSY active" : "recent LoRa activity";
    Serial.printf("DEBUG: Delaying sleep - %s (%d ms ago, <%d ms threshold)\n", 
                  reason, time_since_lora, lora_delay_ms);
    
    // Update activity timestamp if radio is currently busy (less aggressive)
    if (lora_busy) {
      last_lora_activity = now;
    }
    
    return true; // Should delay sleep
  }
  
  // Debug: Show when sleep is allowed
  Serial.printf("DEBUG: Sleep allowed - no LoRa activity for %d ms (>%d ms threshold)\n", 
                time_since_lora, lora_delay_ms);
  return false; // OK to sleep
}

// Enter light sleep for short delays (keeps system running)
void NRFSleep::enterLightSleep(uint32_t timeout_ms) {
  // Light sleep for short delays - system stays running
  // Used when we need to wake up quickly or maintain state
  
  // Skip sleep if buzzer is active
#ifdef PIN_BUZZER
  if (rtttl::isPlaying()) {
    return;
  }
#endif

  if (timeout_ms == 0) {
    sd_app_evt_wait();
    return;
  }

  // Simple timeout-based light sleep
  uint32_t sleep_start = millis();
  
  while ((millis() - sleep_start) < timeout_ms) {
    // Check for immediate wake conditions
#ifdef BUTTON_PIN
    if (digitalRead(BUTTON_PIN) == HIGH) {
      return;
    }
#endif

    // Check LoRa activity
    if (digitalRead(LORA_DIO_1) == HIGH || digitalRead(LORA_BUSY) == HIGH) {
      return;
    }
    
#ifdef PIN_BUZZER
    // Check buzzer activity
    if (rtttl::isPlaying()) {
      return;
    }
#endif
    
    if ((millis() - sleep_start) >= timeout_ms) {
      break;
    }
    
    sd_app_evt_wait(); // Light sleep until next event
  }
}



void NRFSleep::markLoraActivity() {
  last_lora_activity = millis();
}

// Hybrid sleep strategy - balances connectivity and power
void NRFSleep::enterHybridSleep(uint32_t sleep_duration_ms, bool ble_connected) {
  // If BLE is connected, always use light sleep to maintain connection
  if (ble_connected) {
    Serial.printf("DEBUG: BLE connected - using light sleep (%d ms)\n", sleep_duration_ms);
    enterLightSleep(sleep_duration_ms);
    return;
  }
  
  // Hybrid strategy when no BLE client connected:
  // - Use light sleep for first N cycles (maintain connectivity windows)
  // - Then use deep sleep for power savings
  // - Reset cycle counter
  
  if (sleep_cycle_count < LIGHT_SLEEP_CYCLES) {
    // Light sleep phase - maintain BLE connectivity
    uint32_t light_sleep_duration = min(sleep_duration_ms, 3000U); // Max 3s light sleep
    Serial.printf("DEBUG: Hybrid cycle %d/%d - light sleep (%d ms) for connectivity\n", 
                  sleep_cycle_count + 1, LIGHT_SLEEP_CYCLES, light_sleep_duration);
    
    enterLightSleep(light_sleep_duration);
    sleep_cycle_count++;
    
  } else {
    // Deep sleep phase - maximum power savings
    Serial.printf("DEBUG: Hybrid cycle %d - deep sleep for power savings\n", sleep_cycle_count + 1);
    
    sleep_cycle_count = 0; // Reset cycle counter
    enterDeepSleep(); // This will reset the system on wake
    // Note: Code after enterDeepSleep() never executes
  }
}

// Enter deep sleep with 5-second light sleep (SoftDevice compatible)
void NRFSleep::enterDeepSleep() {
  Serial.println("DEBUG: Entering deep sleep - 5s light sleep with all wake sources");
  
  // Power down all T1000E peripherals for maximum power savings
  powerDownPeripherals();
  
  // Turn off LED
#ifdef LED_PIN
  digitalWrite(LED_PIN, LOW);
#endif
  
  uint32_t sleep_start = millis();
  
  // Use light sleep with 5-second timeout - compatible with SoftDevice
  // This maintains interrupt capability and auto-wake timer
  enterLightSleep(5000); // 5 seconds
  
  uint32_t actual_sleep_time = millis() - sleep_start;
  
  // Determine wake reason
  const char* wake_reason = "timeout";
  if (actual_sleep_time < 4500) { // Woke up early (before 4.5s)
#ifdef BUTTON_PIN
    if (digitalRead(BUTTON_PIN) == HIGH) wake_reason = "button";
#endif
    // Check LoRa activity
    if (digitalRead(LORA_DIO_1) == HIGH) {
      wake_reason = "lora-dio1";
      markLoraActivity(); // Update activity timestamp
    }
    if (digitalRead(LORA_BUSY) == HIGH) {
      wake_reason = "lora-busy";
      markLoraActivity(); // Update activity timestamp
    }
#ifdef PIN_BUZZER
    if (rtttl::isPlaying()) wake_reason = "buzzer";
#endif
  }
  
  Serial.printf("DEBUG: Deep sleep wake reason: %s (slept %dms, peripherals powered down)\n", 
                wake_reason, actual_sleep_time);
  
  // Reset cycle count after deep sleep
  sleep_cycle_count = 0;
}

// Power down T1000E peripherals for deep sleep (like Meshtastic)
void NRFSleep::powerDownPeripherals() {
#ifdef GPS_VRTC_EN
  digitalWrite(GPS_VRTC_EN, LOW);
#endif
#ifdef GPS_RESET
  digitalWrite(GPS_RESET, LOW);
#endif
#ifdef GPS_SLEEP_INT
  digitalWrite(GPS_SLEEP_INT, LOW);
#endif
#ifdef GPS_RTC_INT
  digitalWrite(GPS_RTC_INT, LOW);
#endif
#ifdef GPS_RESETB
  pinMode(GPS_RESETB, OUTPUT);
  digitalWrite(GPS_RESETB, LOW);
#endif

#ifdef BUZZER_EN
  digitalWrite(BUZZER_EN, LOW);
#endif

#ifdef PIN_3V3_EN
  digitalWrite(PIN_3V3_EN, LOW);
#endif

  Serial.println("DEBUG: T1000E peripherals powered down for maximum power savings");
}

// Calculate maximum LoRa transaction time using actual radio configuration
uint32_t NRFSleep::calculateMaxLoRaTransactionTime() {
  // Use actual radio driver to get time-on-air for different packet sizes
  // This uses the real configured SF, BW, CR instead of hardcoded assumptions
  
  // Fallback if no radio instance available
  if (!radio_instance) {
    Serial.println("DEBUG: No radio instance - using 5s delay");
    return 5000; // Shorter, more reasonable fallback
  }
  
  // Calculate for typical mesh packet sizes
  uint32_t large_packet_time = radio_instance->getEstAirtimeFor(255);  // Max packet (255 bytes)
  
  // Simple calculation: max packet time + retries + safety margin
  uint32_t total_time = (large_packet_time * 4) + 2000; // 4x for retries, +2s safety
  
  Serial.printf("DEBUG: Dynamic LoRa delay: %dms (based on %dms airtime)\n", 
                total_time, large_packet_time);
  
  return total_time;
}

// Get dynamic LoRa sleep delay based on calculated transaction time
uint32_t NRFSleep::getLoRaSleepDelayMs() {
  static uint32_t cached_delay = 0;
  static uint32_t last_calculation = 0;
  
  // Recalculate every 60 seconds or on first call
  uint32_t now = millis();
  if (cached_delay == 0 || (now - last_calculation) > 60000) {
    cached_delay = calculateMaxLoRaTransactionTime();
    last_calculation = now;
  }
  
  return cached_delay;
}

// High-level sleep management - handles all sleep logic
void NRFSleep::attemptSleep() {
  // Skip sleep if buzzer is playing
#ifdef PIN_BUZZER
  if (rtttl::isPlaying()) {
    return;
  }
#endif
  
  // Dynamic sleep duration based on BLE connection status
  uint32_t sleep_duration_ms;
#ifdef MAX_CONTACTS
  if (Bluefruit.connected()) {
    sleep_duration_ms = 1000;  // 1 second when BLE connected - high responsiveness
  } else {
    sleep_duration_ms = 10000; // 10 seconds when not connected - SoftDevice maintains BLE
  }
#else
  sleep_duration_ms = 10000;   // Default 10 seconds if no BLE support
#endif
  
  uint32_t sleep_start = millis();
  
  // Turn off LED before sleep to save power
#ifdef LED_PIN
  digitalWrite(LED_PIN, LOW);
#endif
  
  // Hybrid sleep strategy - balances connectivity and power
  bool ble_connected = false;
#ifdef MAX_CONTACTS
  ble_connected = Bluefruit.connected();
#endif
  
  enterHybridSleep(sleep_duration_ms, ble_connected);
  
  // Only reached if light sleep was used (deep sleep resets system)
  uint32_t actual_sleep_duration = millis() - sleep_start;
  
  // Determine wake reason for light sleep only
  const char* wake_reason = "timeout";
  if (actual_sleep_duration < (sleep_duration_ms - 500)) { // Woke up early
#ifdef BUTTON_PIN
    if (digitalRead(BUTTON_PIN) == HIGH) wake_reason = "button";
#endif
    // Check LoRa activity
    if (digitalRead(LORA_DIO_1) == HIGH) {
      wake_reason = "lora-dio1";
      markLoraActivity(); // Update activity timestamp
    }
    if (digitalRead(LORA_BUSY) == HIGH) {
      wake_reason = "lora-busy";
      markLoraActivity(); // Update activity timestamp
    }
#ifdef PIN_BUZZER
    if (rtttl::isPlaying()) wake_reason = "buzzer-started";
#endif
  }
  Serial.printf("DEBUG: Hybrid sleep wake reason: %s\n", wake_reason);
}

// Complete sleep loop management for NRF boards
void NRFSleep::manageSleepLoop() {
  uint32_t now = millis();
  
  // Check for sleep every 6 seconds
  if (now - NRFSleep::last_sleep_check > 6000) {
    // Check if sleep should be delayed due to LoRa activity
    if (shouldDelayForLora()) {
      NRFSleep::last_sleep_check = now;  // Update timing but don't sleep
      return;
    }
    
    // Attempt to enter hybrid sleep
    attemptSleep();
    
    // Update sleep check timing after successful sleep attempt
    NRFSleep::last_sleep_check = now;
  }
}

#endif // NRF52_PLATFORM 