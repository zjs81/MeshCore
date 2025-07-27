#ifdef XIAO_NRF52

#include <Arduino.h>
#include "XiaoNrf52Board.h"
#include <helpers/NRFSleep.h>

#include <bluefruit.h>
#include <Wire.h>

static BLEDfu bledfu;

static void connect_callback(uint16_t conn_handle)
{
  (void)conn_handle;
  MESH_DEBUG_PRINTLN("BLE client connected");
  
  NRFSleep::notifyBLEActivity();
  NRFSleep::notifyBLEConnectionAttempt();
}

static void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void)conn_handle;
  (void)reason;

  MESH_DEBUG_PRINTLN("BLE client disconnected");
}

// Callback for BLE security parameter requests - indicates incoming connection attempt
static void security_request_callback(uint16_t conn_handle, ble_gap_evt_sec_params_request_t const * p_request)
{
  (void)conn_handle;
  (void)p_request;
  MESH_DEBUG_PRINTLN("BLE security params requested - connection attempt");
  
  // Notify NRFSleep about connection attempt to prevent sleep for 10 seconds
  NRFSleep::notifyBLEConnectionAttempt();
}

void XiaoNrf52Board::begin() {
  startup_reason = BD_STARTUP_NORMAL;
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  
  pinMode(PIN_VBAT, INPUT);
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, HIGH);

#if defined(PIN_WIRE_SDA) && defined(PIN_WIRE_SCL)
  Wire.setPins(PIN_WIRE_SDA, PIN_WIRE_SCL);
#endif

  Wire.begin();

#ifdef P_LORA_TX_LED
  pinMode(P_LORA_TX_LED, OUTPUT);
  digitalWrite(P_LORA_TX_LED, HIGH);
#endif

//  pinMode(SX126X_POWER_EN, OUTPUT);
//  digitalWrite(SX126X_POWER_EN, HIGH);
  delay(10);   // give sx1262 some time to power up
  
  // Perform one-time ADC calibration for accurate battery monitoring
  NRFAdcCalibration::performBootCalibration();
  
  Serial.printf("DEBUG: XIAO NRF52 - CPU running at %dMHz for power optimization\n", VARIANT_MCK / 1000000);

#ifdef MAX_CONTACTS
  // Initialize BLE callbacks for sleep management
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  
  // Register security callback to catch connection attempts
  Bluefruit.Security.setIOCaps(BLE_GAP_IO_CAPS_NONE);
  Bluefruit.Security.setMITM(false);
  Bluefruit.Security.setBondStorage(false);
  
  Serial.println("DEBUG: BLE callbacks registered for sleep management");
#endif
}

void XiaoNrf52Board::loop() {
  // Complete sleep management handled by NRFSleep
  NRFSleep::manageSleepLoop();
}

bool XiaoNrf52Board::startOTAUpdate(const char* id, char reply[]) {
  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.configPrphConn(92, BLE_GAP_EVENT_LENGTH_MIN, 16, 16);

  Bluefruit.begin(1, 0);
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  // Set the BLE device name
  Bluefruit.setName("XIAO_NRF52_OTA");

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


  return false;
}

#endif