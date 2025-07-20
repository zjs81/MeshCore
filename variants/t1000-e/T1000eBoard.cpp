#include <Arduino.h>
#include "T1000eBoard.h"
#include "SimpleHardwareTimer.h"
#include <Wire.h>
#include <bluefruit.h>
#ifdef PIN_BUZZER
#include <NonBlockingRtttl.h>
#endif

// LoRa activity tracking
static volatile uint32_t last_lora_activity = 0;
static const uint32_t LORA_SLEEP_DELAY_MS = 20000; // 20 seconds

// LoRa DIO1 interrupt handler - captures TX/RX/Timeout events
void lora_dio1_interrupt() {
  // Update activity timestamp (volatile access for interrupt safety)
  uint32_t current_time = millis();
  last_lora_activity = current_time;
  
  // Optional: Could add debug logging here if needed
  // Note: Keep interrupt handler minimal for best performance
}

// Include MyMesh for BLE debug logging (only when companion radio is built)
#ifdef MAX_CONTACTS
#include "../../examples/companion_radio/MyMesh.h"

// BLE event callbacks for proper advertising management
void ble_connect_callback(uint16_t conn_handle) {
  Serial.println("DEBUG: BLE client connected - stopping advertising");
  Bluefruit.Advertising.stop();
}

void ble_disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  Serial.printf("DEBUG: BLE client disconnected (reason: %d) - restarting advertising\n", reason);
  Bluefruit.Advertising.start(0);
}
#endif

void T1000eBoard::begin() {
  // for future use, sub-classes SHOULD call this from their begin()
  startup_reason = BD_STARTUP_NORMAL;
  btn_prev_state = HIGH;

  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);

#ifdef BUTTON_PIN
  pinMode(BATTERY_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
#endif

#if defined(PIN_BOARD_SDA) && defined(PIN_BOARD_SCL)
  Wire.setPins(PIN_BOARD_SDA, PIN_BOARD_SCL);
#endif

  Wire.begin();
  SimpleHardwareTimer::init();
  
  // Setup LoRa DIO1 interrupt for activity tracking
  pinMode(33, INPUT); // LORA_DIO_1 pin
  
  // Initialize LoRa activity timestamp to allow immediate sleep on first boot
  last_lora_activity = 0;
  
  // Attach interrupt for LoRa events (TxDone, RxDone, RxTimeout, etc.)
  attachInterrupt(digitalPinToInterrupt(33), lora_dio1_interrupt, CHANGE);
  Serial.println("DEBUG: LoRa DIO1 interrupt configured for sleep management (CHANGE trigger)");
  
#ifdef MAX_CONTACTS
  // Register BLE callbacks for proper advertising management
  Bluefruit.Periph.setConnectCallback(ble_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(ble_disconnect_callback);
  Serial.println("DEBUG: BLE callbacks registered for advertising management");
#endif
  
  delay(10);   // give sx1262 some time to power up
}

#if 0
static BLEDfu bledfu;

static void connect_callback(uint16_t conn_handle) {
  (void)conn_handle;
  MESH_DEBUG_PRINTLN("BLE client connected");
}

static void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;

  MESH_DEBUG_PRINTLN("BLE client disconnected");
}


bool TrackerT1000eBoard::startOTAUpdate(const char* id, char reply[]) {
  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.configPrphConn(92, BLE_GAP_EVENT_LENGTH_MIN, 16, 16);

  Bluefruit.begin(1, 0);
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  // Set the BLE device name
  Bluefruit.setName("T1000E_OTA");

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Set up and start advertising
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();

  /* Start Advertising
    - Enable auto advertising if disconnected
    - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
    - Timeout for fast mode is 30 seconds
    - Start(timeout) with timeout = 0 will advertise forever (until connected)

    For recommended advertising interval
    https://developer.apple.com/library/content/qa/qa1931/_index.html
  */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds

  strcpy(reply, "OK - started");
  return true;
}
#endif

void T1000eBoard::loop() {
  static uint32_t last_sleep_check = 0;
  uint32_t now = millis();
  
  // Light sleep check every 2 seconds
  if (now - last_sleep_check > 6000) {
    // Check if LoRa activity occurred within the last 20 seconds
    // Handle millis() overflow safely (though rare - occurs every ~49 days)
    uint32_t time_since_lora = (last_lora_activity == 0) ? LORA_SLEEP_DELAY_MS + 1 : (now - last_lora_activity);
    bool lora_activity_recent = time_since_lora < LORA_SLEEP_DELAY_MS;
    
    // Additional check: Don't sleep if LoRa radio is currently busy
    bool lora_busy = digitalRead(7) == HIGH; // LORA_BUSY pin (P0.7)
    
    if (lora_activity_recent || lora_busy) {
      const char* reason = lora_busy ? "LoRa BUSY active" : "recent LoRa activity";
      Serial.printf("DEBUG: Skipping sleep - %s (%d ms ago, <%d ms threshold)\n", 
                    reason, time_since_lora, LORA_SLEEP_DELAY_MS);
      
      // Update activity timestamp if radio is currently busy
      if (lora_busy) {
        last_lora_activity = now;
      }
      
      last_sleep_check = now;
      return;
    }
    
    #ifdef PIN_BUZZER
    if (!rtttl::isPlaying()) {
      //Serial.println("DEBUG: About to sleep (buzzer mode)");
      
      // Dynamic sleep duration based on BLE connection status
      uint32_t sleep_duration_ms;
      bool was_advertising = false;
      
#ifdef MAX_CONTACTS
      if (Bluefruit.connected()) {
        sleep_duration_ms = 1000; // 1 second when BLE connected - high responsiveness
        Serial.println("DEBUG: BLE connected - using 1s sleep");
        // Note: Advertising already stopped by connect callback
      } else {
        sleep_duration_ms = 10000; // 10 seconds when not connected
        //Serial.println("DEBUG: BLE not connected - using 3s sleep");
        
        // Stop advertising during sleep to save power (if still running)
        if (Bluefruit.Advertising.isRunning()) {
          Bluefruit.Advertising.stop();
          was_advertising = true;
          //Serial.println("DEBUG: Stopped BLE advertising for sleep");
        }
      }
#else
      sleep_duration_ms = 10000; // Default 10 seconds if no BLE support
#endif
      
      uint32_t sleep_start = millis();
      
      // Turn off LED before sleep to save power
      digitalWrite(LED_PIN, LOW);
      
      enterLightSleep(sleep_duration_ms);
      wakeFromSleep();
      uint32_t actual_sleep_duration = millis() - sleep_start;
      
      // Restart advertising if we stopped it
#ifdef MAX_CONTACTS
      if (was_advertising && !Bluefruit.connected()) {
        Bluefruit.Advertising.start(0);
        Serial.println("DEBUG: Restarted BLE advertising after sleep");
      }
      
      // Fallback: Ensure advertising state is correct
      if (!Bluefruit.connected() && !Bluefruit.Advertising.isRunning()) {
        Bluefruit.Advertising.start(0);
        Serial.println("DEBUG: Fallback - restarted advertising (was stopped)");
      } else if (Bluefruit.connected() && Bluefruit.Advertising.isRunning()) {
        Bluefruit.Advertising.stop();
        Serial.println("DEBUG: Fallback - stopped advertising (client connected)");
      }
#endif
      
      // Determine wake reason
      const char* wake_reason = "timeout";
      if (actual_sleep_duration < 9500) { // Woke up early
#ifdef BUTTON_PIN
        if (digitalRead(BUTTON_PIN) == HIGH) wake_reason = "button";
#endif
#ifdef MAX_CONTACTS
        if (digitalRead(PIN_SERIAL1_RX) == LOW) wake_reason = "ble-rx1";
        if (digitalRead(PIN_SERIAL1_TX) == LOW) wake_reason = "ble-tx1";
        if (digitalRead(PIN_SERIAL2_RX) == LOW) wake_reason = "ble-rx2";
        if (digitalRead(PIN_SERIAL2_TX) == LOW) wake_reason = "ble-tx2";
#endif
        // Check LoRa DIO1 activity
        if (digitalRead(33) == HIGH) wake_reason = "lora-dio1";
        // Check LoRa BUSY activity
        if (digitalRead(7) == HIGH) wake_reason = "lora-busy";
        // Check buzzer last to avoid interfering with ACK sounds
        if (rtttl::isPlaying()) wake_reason = "buzzer-started";
      }
      
      //Serial.printf("DEBUG: Woke up from sleep (buzzer mode) - slept for %d ms, reason: %s\n", actual_sleep_duration, wake_reason);
    } else {
      //Serial.println("DEBUG: Skipping sleep - buzzer is playing");
    }
    #else
    //Serial.println("DEBUG: About to sleep (no buzzer mode)");
    
    // Dynamic sleep duration based on BLE connection status
    uint32_t sleep_duration_ms;
    bool was_advertising = false;
    
#ifdef MAX_CONTACTS
    if (Bluefruit.connected()) {
      sleep_duration_ms = 1000; // 1 second when BLE connected
      //Serial.println("DEBUG: BLE connected - using 1s sleep");
      // Note: Advertising already stopped by connect callback
    } else {
      sleep_duration_ms = 10000; // 5 seconds when not connected
      //Serial.println("DEBUG: BLE not connected - using 3s sleep");
      
      // Stop advertising during sleep to save power (if still running)
      if (Bluefruit.Advertising.isRunning()) {
        Bluefruit.Advertising.stop();
        was_advertising = true;
        //Serial.println("DEBUG: Stopped BLE advertising for sleep");
      }
    }
#else
    sleep_duration_ms = 10000; // Default 10 seconds if no BLE support
#endif
    
    uint32_t sleep_start = millis();
    
    // Turn off LED before sleep to save power
    digitalWrite(LED_PIN, LOW);
    
    enterLightSleep(sleep_duration_ms);
    wakeFromSleep();
    uint32_t actual_sleep_duration = millis() - sleep_start;
    
    // Restart advertising if we stopped it
#ifdef MAX_CONTACTS
    if (was_advertising && !Bluefruit.connected()) {
      Bluefruit.Advertising.start(0);
      Serial.println("DEBUG: Restarted BLE advertising after sleep");
    }
    
    // Fallback: Ensure advertising state is correct
    if (!Bluefruit.connected() && !Bluefruit.Advertising.isRunning()) {
      Bluefruit.Advertising.start(0);
      //Serial.println("DEBUG: Fallback - restarted advertising (was stopped)");
    } else if (Bluefruit.connected() && Bluefruit.Advertising.isRunning()) {
      Bluefruit.Advertising.stop();
      //Serial.println("DEBUG: Fallback - stopped advertising (client connected)");
    }
#endif
    
    // Determine wake reason
    const char* wake_reason = "timeout";
    if (actual_sleep_duration < 9500) { // Woke up early
#ifdef BUTTON_PIN
      if (digitalRead(BUTTON_PIN) == HIGH) wake_reason = "button";
#endif
#ifdef MAX_CONTACTS
      if (digitalRead(PIN_SERIAL1_RX) == LOW) wake_reason = "ble-rx1";
      if (digitalRead(PIN_SERIAL1_TX) == LOW) wake_reason = "ble-tx1";
      if (digitalRead(PIN_SERIAL2_RX) == LOW) wake_reason = "ble-rx2";
      if (digitalRead(PIN_SERIAL2_TX) == LOW) wake_reason = "ble-tx2";
#endif
      // Check LoRa DIO1 activity
      if (digitalRead(33) == HIGH) wake_reason = "lora-dio1";
      // Check LoRa BUSY activity
      if (digitalRead(7) == HIGH) wake_reason = "lora-busy";
    }
    
    Serial.printf("DEBUG: Woke up from sleep (no buzzer mode) - slept for %d ms, reason: %s\n", actual_sleep_duration, wake_reason);
    #endif
    
    last_sleep_check = now;
  }
}

void T1000eBoard::enterLightSleep(uint32_t timeout_ms) {
  // Skip sleep if buzzer is active
#ifdef PIN_BUZZER
  if (rtttl::isPlaying()) {
    return;
  }
#endif

  // Configure wake sources
#ifdef BUTTON_PIN
  // Button wake-up
  nrf_gpio_cfg_sense_input(
    digitalPinToInterrupt(BUTTON_PIN),
    NRF_GPIO_PIN_PULLDOWN,
    NRF_GPIO_PIN_SENSE_HIGH
  );
#endif

  // LoRa DIO1 wake-up for radio activity
  nrf_gpio_cfg_sense_input(
    digitalPinToInterrupt(33), // LORA_DIO_1
    NRF_GPIO_PIN_NOPULL,
    NRF_GPIO_PIN_SENSE_HIGH
  );
  
  // LoRa BUSY wake-up for active radio operations
  nrf_gpio_cfg_sense_input(
    digitalPinToInterrupt(7), // LORA_BUSY
    NRF_GPIO_PIN_NOPULL,
    NRF_GPIO_PIN_SENSE_HIGH
  );

  // BLE UART RX wake-up (for incoming BLE data)
#ifdef MAX_CONTACTS  // Only if companion radio (BLE) is enabled
  // BLE UART RX wake-up
  nrf_gpio_cfg_sense_input(
    digitalPinToInterrupt(PIN_SERIAL1_RX),
    NRF_GPIO_PIN_PULLUP,
    NRF_GPIO_PIN_SENSE_LOW
  );
  
  // BLE UART TX wake-up (for outgoing BLE data)
  nrf_gpio_cfg_sense_input(
    digitalPinToInterrupt(PIN_SERIAL1_TX),
    NRF_GPIO_PIN_PULLUP,
    NRF_GPIO_PIN_SENSE_LOW
  );
  
  // Also configure UART2 in case BLE uses it
  nrf_gpio_cfg_sense_input(
    digitalPinToInterrupt(PIN_SERIAL2_RX),
    NRF_GPIO_PIN_PULLUP,
    NRF_GPIO_PIN_SENSE_LOW
  );
  
  nrf_gpio_cfg_sense_input(
    digitalPinToInterrupt(PIN_SERIAL2_TX),
    NRF_GPIO_PIN_PULLUP,
    NRF_GPIO_PIN_SENSE_LOW
  );
#endif

  if (timeout_ms == 0) {
    sd_app_evt_wait();
    return;
  }

  // Controlled sleep loop with timeout handling
  uint32_t sleep_start = millis();
  
  while ((millis() - sleep_start) < timeout_ms) {
    // Check for immediate wake conditions
#ifdef BUTTON_PIN
    if (digitalRead(BUTTON_PIN) == HIGH) {
      return;
    }
#endif

    // Check LoRa DIO1 for radio activity
    if (digitalRead(33) == HIGH) {
      return;
    }
    
    // Check LoRa BUSY for active radio operations
    if (digitalRead(7) == HIGH) {
      return;
    }

#ifdef MAX_CONTACTS
    // Check BLE UART pins for activity
    if (digitalRead(PIN_SERIAL1_RX) == LOW || digitalRead(PIN_SERIAL1_TX) == LOW ||
        digitalRead(PIN_SERIAL2_RX) == LOW || digitalRead(PIN_SERIAL2_TX) == LOW) {
      return;
    }
    
    // Remove BLE connection check since we handle this at higher level
    // and stop advertising during sleep anyway
#endif
    
#ifdef PIN_BUZZER
    // Reduced buzzer checking to avoid interfering with ACK sounds
    static uint32_t last_buzzer_check = 0;
    if (millis() - last_buzzer_check > 100) { // Check every 100ms
      if (rtttl::isPlaying()) {
        return;
      }
      last_buzzer_check = millis();
    }
#endif
    
    if ((millis() - sleep_start) >= timeout_ms) {
      break;
    }
    
    sd_app_evt_wait();
  }
}

void T1000eBoard::wakeFromSleep() {
  Bluefruit.Advertising.start(0); // restart advertising

  // Clean up GPIO configuration for wake sources
#ifdef BUTTON_PIN
  nrf_gpio_cfg_input(digitalPinToInterrupt(BUTTON_PIN), NRF_GPIO_PIN_NOPULL);
#endif

  // Reset LoRa DIO1 to normal input
  nrf_gpio_cfg_input(digitalPinToInterrupt(33), NRF_GPIO_PIN_NOPULL);
  
  // Reset LoRa BUSY to normal input
  nrf_gpio_cfg_input(digitalPinToInterrupt(7), NRF_GPIO_PIN_NOPULL);

#ifdef MAX_CONTACTS
  // Reset all BLE UART pins to normal input  
  nrf_gpio_cfg_input(digitalPinToInterrupt(PIN_SERIAL1_RX), NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(digitalPinToInterrupt(PIN_SERIAL1_TX), NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(digitalPinToInterrupt(PIN_SERIAL2_RX), NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(digitalPinToInterrupt(PIN_SERIAL2_TX), NRF_GPIO_PIN_NOPULL);
#endif

  Serial.println("DEBUG: GPIO wake sources cleaned up");
}




