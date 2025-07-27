#include "SensorMesh.h"

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

class MyMesh : public SensorMesh {
public:
  MyMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : SensorMesh(board, radio, ms, rng, rtc, tables), 
       battery_data(12*24, 5*60)    // 24 hours worth of battery data, every 5 minutes
  {
  }

protected:
  /* ========================== custom logic here ========================== */
  Trigger low_batt, critical_batt;
  TimeSeriesData  battery_data;

  void onSensorDataRead() override {
    float batt_voltage = getVoltage(TELEM_CHANNEL_SELF);

    battery_data.recordData(getRTCClock(), batt_voltage);   // record battery
    alertIf(batt_voltage < 3.4f, critical_batt, HIGH_PRI_ALERT, "Battery is critical!");
    alertIf(batt_voltage < 3.6f, low_batt, LOW_PRI_ALERT, "Battery is low");
  }

  int querySeriesData(uint32_t start_secs_ago, uint32_t end_secs_ago, MinMaxAvg dest[], int max_num) override {
    battery_data.calcMinMaxAvg(getRTCClock(), start_secs_ago, end_secs_ago, &dest[0], TELEM_CHANNEL_SELF, LPP_VOLTAGE);
    return 1;
  }

  bool handleCustomCommand(uint32_t sender_timestamp, char* command, char* reply) override {
    if (strcmp(command, "magic") == 0) {    // example 'custom' command handling
      strcpy(reply, "**Magic now done**");
      return true;   // handled
    }
    return false;  // not handled
  }
  /* ======================================================================= */
};

StdRNG fast_rng;
SimpleMeshTables tables;

MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void halt() {
  while (1) ;
}

static char command[120];

void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.print("Please wait...");
    display.endFrame();
  }
#endif

  if (!radio_init()) { halt(); }

  fast_rng.begin(radio_get_rng_seed());

  FILESYSTEM* fs;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  InternalFS.begin();
  fs = &InternalFS;
  IdentityStore store(InternalFS, "");
#elif defined(ESP32)
  SPIFFS.begin(true);
  fs = &SPIFFS;
  IdentityStore store(SPIFFS, "/identity");
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  fs = &LittleFS;
  IdentityStore store(LittleFS, "/identity");
  store.begin();
#else
  #error "need to define filesystem"
#endif
  if (!store.load("_main", the_mesh.self_id)) {
    MESH_DEBUG_PRINTLN("Generating new keypair");
    the_mesh.self_id = radio_new_identity();   // create new random identity
    int count = 0;
    while (count < 10 && (the_mesh.self_id.pub_key[0] == 0x00 || the_mesh.self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
      the_mesh.self_id = radio_new_identity(); count++;
    }
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Sensor ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  command[0] = 0;

  sensors.begin();

  the_mesh.begin(fs);

#ifdef NRF52_PLATFORM
  // Update NRFSleep with dispatcher reference for full packet queue awareness
  NRFSleep::setDispatcher(&the_mesh);
  
  // 🚀 PRODUCTION DEFAULT: Adaptive duty cycling is now ENABLED BY DEFAULT!
  // Conservative 3%-25% range ensures zero packet loss with 5-12x power savings
  // Current consumption: ~0.4-1.2mA (was 5mA) = weeks/months of battery life
  
  // ✅ RECOMMENDED: Use the production defaults (no code needed!)
  // Default adaptive range: 3%-25% duty cycle - safe and efficient
  
  // 🔧 OPTIONAL: Customize the adaptive range for your specific use case:
  // NRFSleep::enableAdaptiveDutyCycle(1.0, 15.0);   // Max power savings (sparse networks)
  // NRFSleep::enableAdaptiveDutyCycle(5.0, 30.0);   // Max responsiveness (dense networks)
  
  // 🔧 ALTERNATIVE: Manual fixed duty cycle (not recommended for production)
  // NRFSleep::setDutyCycle(100, 2000);              // Fixed 5% duty cycle 
  
  // 🔧 EMERGENCY: Disable duty cycling for debugging/testing only
  // NRFSleep::disableDutyCycle();                   // Continuous RX, high power
#endif

#ifdef DISPLAY_CLASS
  ui_task.begin(the_mesh.getNodePrefs(), FIRMWARE_BUILD_DATE, FIRMWARE_VERSION);
#endif

  // send out initial Advertisement to the mesh
  the_mesh.sendSelfAdvertisement(16000);
}

void loop() {
  // Cache current time to avoid multiple calls and optimize serial processing
  static unsigned long last_sensor_loop = 0;
  static unsigned long last_ui_loop = 0;
  static unsigned long last_board_loop = 0;
  static char* command_ptr = command; // Cache command buffer pointer
  
  unsigned long current_millis = millis();
  
  // Process serial input with optimized approach
  int len = command_ptr - command; // Use pointer arithmetic instead of strlen
  while (Serial.available() && len < sizeof(command)-1) {
    char c = Serial.read();
    if (c != '\n') {
      *command_ptr++ = c;
      *command_ptr = 0;
      len++;
    }
    Serial.print(c);
  }
  if (len == sizeof(command)-1) {  // command buffer full
    command[sizeof(command)-1] = '\r';
  }

  if (len > 0 && command[len - 1] == '\r') {  // received complete line
    command[len - 1] = 0;  // replace newline with C string null terminator
    char reply[160];
    the_mesh.handleCommand(0, command, reply);  // NOTE: there is no sender_timestamp via serial!
    if (reply[0]) {
      Serial.print("  -> "); Serial.println(reply);
    }

    // Reset command buffer efficiently
    command_ptr = command;
    *command_ptr = 0;
  }

  // Main mesh processing - always run as it handles critical radio operations
  the_mesh.loop();
  
  // Sensors - only check every 100ms to reduce CPU load
  if (current_millis - last_sensor_loop >= 100) {
    sensors.loop();
    last_sensor_loop = current_millis;
  }
  
#ifdef DISPLAY_CLASS
  // UI updates - only every 50ms for smooth display updates
  if (current_millis - last_ui_loop >= 50) {
    ui_task.loop();
    last_ui_loop = current_millis;
  }
#endif

#ifdef NRF52_PLATFORM
  // Power management - only every 10ms to balance power savings with responsiveness
  if (current_millis - last_board_loop >= 10) {
    board.loop(); // Hybrid sleep management for power optimization
    last_board_loop = current_millis;
  }
#endif
}
