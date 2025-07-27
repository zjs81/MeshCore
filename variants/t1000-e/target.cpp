#include <Arduino.h>
#include "t1000e_sensors.h"
#include "target.h"
#include <helpers/sensors/MicroNMEALocationProvider.h>

T1000eBoard board;

RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, SPI);

WRAPPER_CLASS radio_driver(radio, board);

VolatileRTCClock rtc_clock;
MicroNMEALocationProvider nmea = MicroNMEALocationProvider(Serial1, &rtc_clock);
T1000SensorManager sensors = T1000SensorManager(nmea);

#ifdef DISPLAY_CLASS
  NullDisplayDriver display;
#endif

#ifndef LORA_CR
  #define LORA_CR      5
#endif

#ifdef RF_SWITCH_TABLE
static const uint32_t rfswitch_dios[Module::RFSWITCH_MAX_PINS] = {
  RADIOLIB_LR11X0_DIO5,
  RADIOLIB_LR11X0_DIO6,
  RADIOLIB_LR11X0_DIO7,
  RADIOLIB_LR11X0_DIO8, 
  RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table[] = {
  // mode                 DIO5  DIO6  DIO7  DIO8
  { LR11x0::MODE_STBY,   {LOW,  LOW,  LOW,  LOW  }},  
  { LR11x0::MODE_RX,     {HIGH, LOW,  LOW,  HIGH }},
  { LR11x0::MODE_TX,     {HIGH, HIGH, LOW,  HIGH }},
  { LR11x0::MODE_TX_HP,  {LOW,  HIGH, LOW,  HIGH }},
  { LR11x0::MODE_TX_HF,  {LOW,  LOW,  LOW,  LOW  }}, 
  { LR11x0::MODE_GNSS,   {LOW,  LOW,  HIGH, LOW  }},
  { LR11x0::MODE_WIFI,   {LOW,  LOW,  LOW,  LOW  }},  
  END_OF_MODE_TABLE,
};
#endif

bool radio_init() {
  //rtc_clock.begin(Wire);
  
#ifdef LR11X0_DIO3_TCXO_VOLTAGE
  float tcxo = LR11X0_DIO3_TCXO_VOLTAGE;
#else
  float tcxo = 1.6f;
#endif

  SPI.setPins(P_LORA_MISO, P_LORA_SCLK, P_LORA_MOSI);
  SPI.begin();
  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_LR11X0_LORA_SYNC_WORD_PRIVATE, LORA_TX_POWER, 16, tcxo);
  if (status != RADIOLIB_ERR_NONE) {
    Serial.print("ERROR: radio init failed: ");
    Serial.println(status);
    return false;  // fail
  }
  
  radio.setCRC(2);
  radio.explicitHeader();

#ifdef RF_SWITCH_TABLE
  radio.setRfSwitchTable(rfswitch_dios, rfswitch_table);
#endif
#ifdef RX_BOOSTED_GAIN
  radio.setRxBoostedGainMode(RX_BOOSTED_GAIN);
#endif

  return true;  // success
}

uint32_t radio_get_rng_seed() {
  return radio.random(0x7FFFFFFF);
}

void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr) {
  radio.setFrequency(freq);
  radio.setSpreadingFactor(sf);
  radio.setBandwidth(bw);
  radio.setCodingRate(cr);
}

void radio_set_tx_power(uint8_t dbm) {
  radio.setOutputPower(dbm);
}

mesh::LocalIdentity radio_new_identity() {
  RadioNoiseListener rng(radio);
  return mesh::LocalIdentity(&rng);  // create new random identity
}

void T1000SensorManager::start_gps() {
  gps_active = true;
  //_nmea->begin();
  // this init sequence should be better 
  // comes from seeed examples and deals with all gps pins
  pinMode(GPS_EN, OUTPUT);
  digitalWrite(GPS_EN, HIGH);
  delay(10);
  pinMode(GPS_VRTC_EN, OUTPUT);
  digitalWrite(GPS_VRTC_EN, HIGH);
  delay(10);
       
  pinMode(GPS_RESET, OUTPUT);
  digitalWrite(GPS_RESET, HIGH);
  delay(10);
  digitalWrite(GPS_RESET, LOW);
       
  pinMode(GPS_SLEEP_INT, OUTPUT);
  digitalWrite(GPS_SLEEP_INT, HIGH);
  pinMode(GPS_RTC_INT, OUTPUT);
  digitalWrite(GPS_RTC_INT, LOW);
  pinMode(GPS_RESETB, INPUT_PULLUP);
}

void T1000SensorManager::sleep_gps() {
  gps_active = false;
  digitalWrite(GPS_VRTC_EN, HIGH);
  digitalWrite(GPS_EN, LOW);
  digitalWrite(GPS_RESET, HIGH);
  digitalWrite(GPS_SLEEP_INT, HIGH);
  digitalWrite(GPS_RTC_INT, LOW);
  pinMode(GPS_RESETB, OUTPUT);
  digitalWrite(GPS_RESETB, LOW);
  //_nmea->stop();
}

void T1000SensorManager::stop_gps() {
  gps_active = false;
  digitalWrite(GPS_VRTC_EN, LOW);
  digitalWrite(GPS_EN, LOW);
  digitalWrite(GPS_RESET, HIGH);
  digitalWrite(GPS_SLEEP_INT, HIGH);
  digitalWrite(GPS_RTC_INT, LOW);
  pinMode(GPS_RESETB, OUTPUT);
  digitalWrite(GPS_RESETB, LOW);
  //_nmea->stop();
}


bool T1000SensorManager::begin() {
  // init GPS
  Serial1.begin(115200);

  // make sure gps pin are off
  digitalWrite(GPS_VRTC_EN, LOW);
  digitalWrite(GPS_RESET, LOW);
  digitalWrite(GPS_SLEEP_INT, LOW);
  digitalWrite(GPS_RTC_INT, LOW);
  pinMode(GPS_RESETB, OUTPUT);
  digitalWrite(GPS_RESETB, LOW);

  // Initialize sensor cache with first reading (optimized single power cycle)
  Serial.println("ENV: Initial sensor reading");
  T1000eSensorReading initial_reading = t1000e_get_sensors_combined();
  cached_temperature = initial_reading.temperature;
  cached_light = initial_reading.light;
  last_sensor_read = millis();
  Serial.printf("ENV: Initial values - Temperature: %.1f°C, Light: %u%%\n", 
               cached_temperature, cached_light);

  return true;
}

bool T1000SensorManager::querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) {
  if (requester_permissions & TELEM_PERM_LOCATION) {   // does requester have permission?
    telemetry.addGPS(TELEM_CHANNEL_SELF, node_lat, node_lon, node_altitude);
  }
  if (requester_permissions & TELEM_PERM_ENVIRONMENT) {
    // Use cached sensor values instead of reading every time
    telemetry.addLuminosity(TELEM_CHANNEL_SELF, cached_light);
    telemetry.addTemperature(TELEM_CHANNEL_SELF, cached_temperature);
  }
  return true;
}

void T1000SensorManager::loop() {
  static long next_gps_update = 0;
  static uint32_t last_gps_cycle = 0;

  _nmea->loop();

  if (millis() > next_gps_update) {
    if (gps_active && _nmea->isValid()) {
      node_lat = ((double)_nmea->getLatitude())/1000000.;
      node_lon = ((double)_nmea->getLongitude())/1000000.;
      node_altitude = ((double)_nmea->getAltitude()) / 1000.0;
      //Serial.printf("lat %f lon %f\r\n", _lat, _lon);
    }
    next_gps_update = millis() + 1000;
  }

  // Update environmental sensors based on battery-aware schedule
  updateEnvironmentalSensors();

  // GPS power cycling for battery optimization - only when GPS is active
  if (gps_active) {
    uint32_t now = millis();
    if (now - last_gps_cycle > 300000) { // 5 minutes
      cycleGpsPower();
      last_gps_cycle = now;
    }
  }
}

int T1000SensorManager::getNumSettings() const { return 1; }  // just one supported: "gps" (power switch)

const char* T1000SensorManager::getSettingName(int i) const {
  return i == 0 ? "gps" : NULL;
}
const char* T1000SensorManager::getSettingValue(int i) const {
  if (i == 0) {
    return gps_active ? "1" : "0";
  }
  return NULL;
}
bool T1000SensorManager::setSettingValue(const char* name, const char* value) {
  if (strcmp(name, "gps") == 0) {
    if (strcmp(value, "0") == 0) {
      sleep_gps(); // sleep for faster fix !
    } else {
      start_gps();
    }
    return true;
  }
  return false;  // not supported
}

uint32_t T1000SensorManager::getGPSSleepDuration() {
  // Base sleep duration: 5 minutes
  uint32_t base_sleep = 300000;
  
  // Get battery level (if available)
  float battery_voltage = board.getBattMilliVolts() / 1000.0f;
  
  // Scale sleep duration based on battery level
  if (battery_voltage < 3.3f) {        // Very low battery
    return base_sleep * 4;              // 20 minutes
  } else if (battery_voltage < 3.6f) {  // Low battery  
    return base_sleep * 2;              // 10 minutes
  } else if (battery_voltage < 3.8f) {  // Medium battery
    return base_sleep * 1.5f;           // 7.5 minutes
  } else {                              // Good battery
    return base_sleep;                  // 5 minutes
  }
}

bool T1000SensorManager::shouldSkipGPSCycle() {
  // Skip GPS entirely when battery is critically low
  float battery_voltage = board.getBattMilliVolts() / 1000.0f;
  
  if (battery_voltage < 3.2f) {
    // Critical battery - skip GPS for 12 minute cycles maximum
    static uint32_t last_critical_skip = 0;
    uint32_t now = millis();
    
    if (now - last_critical_skip > 720000) {  // 12 minutes
      last_critical_skip = now;
      return false;  // Allow one GPS cycle every 12 minutes
    }
    return true;  // Skip this cycle
  }
  
  return false;  // Normal operation
}

uint32_t T1000SensorManager::getSensorReadInterval() {
  // Base sensor reading interval: 1 minute (60 seconds)
  uint32_t base_interval = 60000;
  
  // Get battery level (if available)
  float battery_voltage = board.getBattMilliVolts() / 1000.0f;
  
  // Scale sensor reading interval based on battery level
  if (battery_voltage < 3.3f) {        // Very low battery
    return base_interval * 10;          // 10 minutes
  } else if (battery_voltage < 3.6f) {  // Low battery  
    return base_interval * 5;           // 5 minutes
  } else if (battery_voltage < 3.8f) {  // Medium battery
    return base_interval * 2;           // 2 minutes
  } else {                              // Good battery
    return base_interval;               // 1 minute
  }
}

void T1000SensorManager::updateEnvironmentalSensors() {
  uint32_t now = millis();
  uint32_t read_interval = getSensorReadInterval();
  
  // Check if it's time to read sensors
  if (now - last_sensor_read >= read_interval) {
    Serial.printf("ENV: Reading sensors (battery: %.2fV, interval: %us)\n", 
                 board.getBattMilliVolts() / 1000.0f, read_interval/1000);
    
    // Read both sensors in optimized single power cycle
    T1000eSensorReading reading = t1000e_get_sensors_combined();
    cached_temperature = reading.temperature;
    cached_light = reading.light;
    
    last_sensor_read = now;
    
    Serial.printf("ENV: Temperature: %.1f°C, Light: %u%%\n", 
                 cached_temperature, cached_light);
  }
}

void T1000SensorManager::cycleGpsPower() {
  // Check if we should skip GPS due to low battery
  if (shouldSkipGPSCycle()) {
    return;
  }
  
  // Smart GPS power cycling: Quick acquisition -> battery-aware sleep
  static enum {
    GPS_SLEEPING,     // GPS off, waiting for next cycle
    GPS_ACQUIRING,    // GPS on, waiting for fix
    GPS_TRACKING      // GPS has fix, ready to sleep
  } gps_state = GPS_SLEEPING;
  
  static uint32_t state_start_time = 0;
  static uint32_t acquisition_timeout = 120000;  // 2 minutes max acquisition
  static uint32_t last_successful_fix = 0;
  
  uint32_t now = millis();
  uint32_t state_duration = now - state_start_time;
  uint32_t sleep_duration = getGPSSleepDuration();  // Battery-aware sleep duration
  
  switch(gps_state) {
    case GPS_SLEEPING:
      if (state_duration >= sleep_duration) {
        // Time to wake up and get a new fix
        Serial.printf("GPS: Starting acquisition cycle (battery: %.2fV)\n", 
                     board.getBattMilliVolts() / 1000.0f);
        start_gps();
        gps_state = GPS_ACQUIRING;
        state_start_time = now;
        
        // Adaptive acquisition timeout based on time since last fix
        uint32_t time_since_fix = now - last_successful_fix;
        if (time_since_fix > 7200000) {  // >2 hours = cold start
          acquisition_timeout = 180000;  // 3 minutes for cold start
        } else if (time_since_fix > 1800000) {  // >30 minutes = warm start
          acquisition_timeout = 90000;   // 1.5 minutes for warm start  
        } else {
          acquisition_timeout = 60000;   // 1 minute for hot start
        }
      }
      break;
      
    case GPS_ACQUIRING:
      if (_nmea->isValid()) {
        // Got a fix! Switch to tracking mode briefly to ensure stability
        Serial.println("GPS: Fix acquired, entering tracking mode");
        gps_state = GPS_TRACKING;
        state_start_time = now;
        last_successful_fix = now;
      } else if (state_duration >= acquisition_timeout) {
        // Timeout - give up and sleep
        Serial.printf("GPS: Acquisition timeout after %ums, sleeping\n", state_duration);
        sleep_gps();
        gps_state = GPS_SLEEPING;
        state_start_time = now;
      }
      break;
      
    case GPS_TRACKING:
      if (state_duration >= 10000) {  // Track for 10 seconds to ensure fix stability
        Serial.printf("GPS: Cycle complete, sleeping for %u minutes\n", sleep_duration/60000);
        sleep_gps();
        gps_state = GPS_SLEEPING;
        state_start_time = now;
      }
      break;
  }
}
