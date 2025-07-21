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

// Include MyMesh for BLE debug logging (only when companion radio is built)
#ifdef MAX_CONTACTS
#include "../../examples/companion_radio/MyMesh.h"

// Note: SoftDevice automatically handles BLE advertising during sleep
// No manual callbacks needed - hardware manages advertising state
#endif

void T1000eBoard::begin() {
  // for future use, sub-classes SHOULD call this from their begin()
  startup_reason = BD_STARTUP_NORMAL;
  btn_prev_state = HIGH;

  // Set low power mode and optimize CPU frequency
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  
  // CPU now running at 32MHz (configured in variant.h) for power optimization
  Serial.printf("DEBUG: CPU running at %dMHz for power optimization\n", VARIANT_MCK / 1000000);

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
  
  // Initialize NRF sleep management with LoRa interrupt handling
  NRFSleep::init();
  
#ifdef MAX_CONTACTS
  // SoftDevice automatically handles BLE advertising during sleep
  // No manual callback registration needed - hardware manages this
  Serial.println("DEBUG: BLE enabled - SoftDevice will maintain advertising during sleep");
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
  Bluefruit.Advertising.setInterval(32, 8000); // 20ms fast mode, 5.0s slow mode (in unit of 0.625 ms)
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds

  strcpy(reply, "OK - started");
  return true;
}
#endif

void T1000eBoard::loop() {
  // Complete sleep management handled by NRFSleep
  NRFSleep::manageSleepLoop();
}

