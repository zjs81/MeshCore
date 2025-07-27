#include <Arduino.h>
#include "T1000eBoard.h"
#include "variant.h"
#include <helpers/SimpleHardwareTimer.h>
#include <helpers/NRFSleep.h>
#include <Wire.h>
#include <bluefruit.h>
#ifdef PIN_BUZZER
#include <NonBlockingRtttl.h>
#endif
#include <helpers/NRFAdcCalibration.h>

// Define button pressed state if not already defined by platformio.ini
#ifndef USER_BTN_PRESSED
#define USER_BTN_PRESSED LOW  // Default for most boards
#endif

// Include MyMesh for BLE debug logging (only when companion radio is built)
#ifdef MAX_CONTACTS
#include "../../examples/companion_radio/MyMesh.h"

// Note: SoftDevice automatically handles BLE advertising during sleep
// No manual callbacks needed - hardware manages advertising state
#endif

// BLE connection callbacks for sleep management
static void connect_callback(uint16_t conn_handle) {
  (void)conn_handle;
  MESH_DEBUG_PRINTLN("BLE client connected");
  
  // Notify NRFSleep about BLE activity to prevent sleep for 10 seconds
  NRFSleep::notifyBLEActivity();
  
  // Also notify about connection attempt to extend wake time
  NRFSleep::notifyBLEConnectionAttempt();
}

static void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;
  MESH_DEBUG_PRINTLN("BLE client disconnected");
}

void T1000eBoard::begin() {
  // for future use, sub-classes SHOULD call this from their begin()
  startup_reason = BD_STARTUP_NORMAL;
  btn_prev_state = !USER_BTN_PRESSED;  // Initialize to not-pressed state
  btn_press_start = 0;
  btn_long_press_triggered = false;
  power_button_enabled = true; // Set to false to disable power button shutdown

  // Set low power mode and optimize CPU frequency
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  
  // CPU now running at 32MHz (configured in variant.h) for power optimization
  Serial.printf("DEBUG: CPU running at %dMHz for power optimization\n", VARIANT_MCK / 1000000);

#ifdef BUTTON_PIN
  pinMode(BATTERY_PIN, INPUT);
  // Configure button pin based on type: INPUT_PULLUP for active LOW, INPUT for active HIGH (hardware handles pullup/pulldown)
  #if USER_BTN_PRESSED == LOW
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // Active LOW button needs pullup
  #else
    pinMode(BUTTON_PIN, INPUT);  // Active HIGH button - hardware handles pullup/pulldown
  #endif
  pinMode(LED_PIN, OUTPUT);
#endif

#if defined(PIN_BOARD_SDA) && defined(PIN_BOARD_SCL)
  Wire.setPins(PIN_BOARD_SDA, PIN_BOARD_SCL);
#endif

  Wire.begin();
  SimpleHardwareTimer::init();
  
  NRFAdcCalibration::performBootCalibration();
  
  float factor;
  NRFAdcCalibration::CalibrationMethod method;
  float accuracy;
  const char* methodName = NRFAdcCalibration::getCalibrationInfo(factor, method, accuracy);
  Serial.printf("DEBUG T1000E: ADC Calibration - %s (Factor: %.4f, Accuracy: %.1f%%)\n", 
                methodName, factor, accuracy);
  
#ifdef MAX_CONTACTS
  // SoftDevice automatically handles BLE advertising during sleep
  // No manual callback registration needed - hardware manages this
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  Serial.println("DEBUG: T1000-E - BLE advertising power optimization enabled");

  // Initialize BLE callbacks for sleep management
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  
  Serial.println("DEBUG: BLE callbacks registered for sleep management");
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
  Bluefruit.setTxPower(4);
  Bluefruit.setName("T1000E_OTA");

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  bledfu.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 8000);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);

  strcpy(reply, "OK - started");
  return true;
}
#endif

void T1000eBoard::loop() {
  static unsigned long last_button_check = 0;
  unsigned long now = millis();
  if (now - last_button_check >= 100) {
    buttonStateChanged();
    last_button_check = now;
  }
  
  // Complete sleep management handled by NRFSleep
  NRFSleep::manageSleepLoop();
}

