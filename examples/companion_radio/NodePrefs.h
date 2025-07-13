#pragma once
#include <cstdint> // For uint8_t, uint32_t

#define TELEM_MODE_DENY            0
#define TELEM_MODE_ALLOW_FLAGS     1     // use contact.flags
#define TELEM_MODE_ALLOW_ALL       2

#define ADVERT_LOC_NONE       0
#define ADVERT_LOC_SHARE      1

struct NodePrefs {  // persisted to file
  float airtime_factor;
  char node_name[32];
  float freq;
  uint8_t sf;
  uint8_t cr;
  uint8_t reserved1;
  uint8_t manual_add_contacts;
  float bw;
  uint8_t tx_power_dbm;
  uint8_t telemetry_mode_base;
  uint8_t telemetry_mode_loc;
  uint8_t telemetry_mode_env;
  float rx_delay_base;
  uint32_t ble_pin;
  uint8_t  advert_loc_policy;
  uint8_t auto_time_sync;
  uint8_t time_sync_max_hops;
  uint8_t time_sync_min_samples;
  uint16_t time_sync_max_drift;
  uint8_t time_req_pool_size;
  uint16_t time_req_slew_limit;
  uint8_t time_req_min_samples;
  float time_sync_alpha;
  float time_sync_max_drift_rate;
  float time_sync_tolerance;
};