#pragma once

#include "Mesh.h"
#include <helpers/IdentityStore.h>

struct NodePrefs {  // persisted to file
    float airtime_factor;
    char node_name[32];
    double node_lat, node_lon;
    char password[16];
    float freq;
    uint8_t tx_power_dbm;
    uint8_t disable_fwd;
    uint8_t advert_interval;   // minutes / 2
    uint8_t flood_advert_interval;   // hours
    float rx_delay_base;
    float tx_delay_factor;
    char guest_password[16];
    float direct_tx_delay_factor;
    uint32_t guard;
    uint8_t sf;
    uint8_t cr;
    uint8_t allow_read_only;
    uint8_t reserved2;
    float bw;
    uint8_t flood_max;
    uint8_t interference_threshold;
    uint8_t agc_reset_interval;   // secs / 4
    uint8_t auto_time_sync;       // enable/disable automatic time sync
    uint8_t time_sync_max_hops;   // max hops to accept time from (default: 6)
    uint8_t time_sync_min_samples; // min consistent samples before sync (default: 3)
    uint16_t time_sync_max_drift;  // max allowed time drift in seconds (default: 3600)
    uint8_t time_req_pool_size;   // pool size for time requests (default: 0 = auto)
    uint16_t time_req_slew_limit; // slew rate limit in seconds (default: 5)
    uint8_t time_req_min_samples; // min samples for time requests (default: 3)
    float time_sync_alpha;        // smoothing factor for drift rate (default: 0.1)
    float time_sync_max_drift_rate; // maximum drift rate in s/s (default: 0.01)
    float time_sync_tolerance;    // tolerance for plausibility check in seconds (default: 5.0)
};

class CommonCLICallbacks {
public:
  virtual void savePrefs() = 0;
  virtual const char* getFirmwareVer() = 0;
  virtual const char* getBuildDate() = 0;
  virtual const char* getRole() = 0;
  virtual bool formatFileSystem() = 0;
  virtual void sendSelfAdvertisement(int delay_millis) = 0;
  virtual void updateAdvertTimer() = 0;
  virtual void updateFloodAdvertTimer() = 0;
  virtual void setLoggingOn(bool enable) = 0;
  virtual void eraseLogFile() = 0;
  virtual void dumpLogFile() = 0;
  virtual void setTxPower(uint8_t power_dbm) = 0;
  virtual void formatNeighborsReply(char *reply) = 0;
  virtual const uint8_t* getSelfIdPubKey() = 0;
  virtual void clearStats() = 0;
};

class CommonCLI {
  mesh::RTCClock* _rtc;
  NodePrefs* _prefs;
  CommonCLICallbacks* _callbacks;
  mesh::MainBoard* _board;
  char tmp[80];

  mesh::RTCClock* getRTCClock() { return _rtc; }
  void savePrefs();
  void loadPrefsInt(FILESYSTEM* _fs, const char* filename);

public:
  CommonCLI(mesh::MainBoard& board, mesh::RTCClock& rtc, NodePrefs* prefs, CommonCLICallbacks* callbacks)
      : _board(&board), _rtc(&rtc), _prefs(prefs), _callbacks(callbacks) { }

  void loadPrefs(FILESYSTEM* _fs);
  void savePrefs(FILESYSTEM* _fs);
  void handleCommand(uint32_t sender_timestamp, const char* command, char* reply);
};
