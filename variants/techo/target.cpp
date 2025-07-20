#include <Arduino.h>
#include "target.h"
#include <helpers/ArduinoHelpers.h>
#include <helpers/sensors/MicroNMEALocationProvider.h>
#include <helpers/NRFSleep.h>

TechoBoard board;

RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, SPI);

WRAPPER_CLASS radio_driver(radio, board);

VolatileRTCClock fallback_clock;
AutoDiscoverRTCClock rtc_clock(fallback_clock);
MicroNMEALocationProvider nmea = MicroNMEALocationProvider(Serial1);
TechoSensorManager sensors = TechoSensorManager(nmea);

#ifdef DISPLAY_CLASS
  DISPLAY_CLASS display;
#endif

bool radio_init() {
  rtc_clock.begin(Wire);
  bool success = radio.std_init(&SPI);
  
  // Initialize NRF sleep management with radio instance for proper airtime calculations
  if (success) {
    NRFSleep::init(&radio_driver);
    Serial.println("DEBUG: TEcho - NRFSleep initialized with radio instance");
  }
  
  return success;
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

void TechoSensorManager::start_gps() {
  if (!gps_active) {
    gps_active = true;
    _location->begin();
  }
}

void TechoSensorManager::stop_gps() {
  if (gps_active) {
    gps_active = false;
    _location->stop();
  }
}

bool TechoSensorManager::begin() {
  Serial1.begin(9600);

  // GPS enabled pin
  pinMode(GPS_EN, OUTPUT);
  
  return true;
}

bool TechoSensorManager::querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) {
  if (requester_permissions & TELEM_PERM_LOCATION) {   // does requester have permission?
    telemetry.addGPS(TELEM_CHANNEL_SELF, node_lat, node_lon, node_altitude);
  }
  return true;
}

void TechoSensorManager::loop() {
  static long next_gps_update = 0;
  
  if (!gps_active) {
    return;  // GPS is not active, skip further processing
  }

  _location->loop();

  if (millis() > next_gps_update) {
    if (_location->isValid()) {
      node_lat = ((double)_location->getLatitude())/1000000.;
      node_lon = ((double)_location->getLongitude())/1000000.;
      node_altitude = ((double)_location->getAltitude()) / 1000.0;
      MESH_DEBUG_PRINTLN("lat %f lon %f", node_lat, node_lon);
    }
    next_gps_update = millis() + 1000;
  }
}

int TechoSensorManager::getNumSettings() const {
  return 1;  // always show GPS setting
}

const char* TechoSensorManager::getSettingName(int i) const {
  return (i == 0) ? "gps" : NULL;
}

const char* TechoSensorManager::getSettingValue(int i) const {
  if (i == 0) {
    return gps_active ? "1" : "0";
  }
  return NULL;
}

bool TechoSensorManager::setSettingValue(const char* name, const char* value) {
  if (strcmp(name, "gps") == 0) {
    if (strcmp(value, "0") == 0) {
      stop_gps();
    } else {
      start_gps();
    }
    return true;
  }
  return false;  // not supported
}
