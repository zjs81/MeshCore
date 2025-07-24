#pragma once

#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#include "TimeSeriesData.h"

#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
#include <InternalFileSystem.h>
#elif defined(RP2040_PLATFORM)
#include <LittleFS.h>
#elif defined(ESP32)
#include <SPIFFS.h>
#endif

#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/IdentityStore.h>
#include <helpers/AdvertDataHelpers.h>
#include <helpers/TxtDataHelpers.h>
#include <helpers/CommonCLI.h>
#include <RTClib.h>
#include <target.h>

#define PERM_ACL_ROLE_MASK     3   // lower 2 bits
#define PERM_ACL_GUEST         0
#define PERM_ACL_READ_ONLY     1
#define PERM_ACL_READ_WRITE    2
#define PERM_ACL_ADMIN         3

#define PERM_RESERVED1         (1 << 2)
#define PERM_RESERVED2         (1 << 3)
#define PERM_RESERVED3         (1 << 4)
#define PERM_RESERVED4         (1 << 5)
#define PERM_RECV_ALERTS_LO    (1 << 6)   // low priority alerts
#define PERM_RECV_ALERTS_HI    (1 << 7)   // high priority alerts

struct ContactInfo {
  mesh::Identity id;
  uint8_t permissions;
  int8_t out_path_len;
  uint8_t out_path[MAX_PATH_SIZE];
  uint8_t shared_secret[PUB_KEY_SIZE];
  uint32_t last_timestamp;   // by THEIR clock  (transient)
  uint32_t last_activity;    // by OUR clock    (transient)

  bool isAdmin() const { return (permissions & PERM_ACL_ROLE_MASK) == PERM_ACL_ADMIN; }
};

#ifndef FIRMWARE_BUILD_DATE
  #define FIRMWARE_BUILD_DATE   "20 Jul 2025"
#endif

#ifndef FIRMWARE_VERSION
  #define FIRMWARE_VERSION   "v1.7.2"
#endif

#define FIRMWARE_ROLE "sensor"

#define MAX_CONTACTS           20

#define MAX_SEARCH_RESULTS      8
#define MAX_CONCURRENT_ALERTS   4

class SensorMesh : public mesh::Mesh, public CommonCLICallbacks {
public:
  SensorMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables);
  void begin(FILESYSTEM* fs);
  void loop();
  void handleCommand(uint32_t sender_timestamp, char* command, char* reply);

  // CommonCLI callbacks
  const char* getFirmwareVer() override { return FIRMWARE_VERSION; }
  const char* getBuildDate() override { return FIRMWARE_BUILD_DATE; }
  const char* getRole() override { return FIRMWARE_ROLE; }
  const char* getNodeName() { return _prefs.node_name; }
  NodePrefs* getNodePrefs() { return &_prefs; }
  void savePrefs() override { _cli.savePrefs(_fs); }
  bool formatFileSystem() override;
  void sendSelfAdvertisement(int delay_millis) override;
  void updateAdvertTimer() override;
  void updateFloodAdvertTimer() override;
  void setLoggingOn(bool enable) override {  }
  void eraseLogFile() override { }
  void dumpLogFile() override { }
  void setTxPower(uint8_t power_dbm) override;
  void formatNeighborsReply(char *reply) override {
    strcpy(reply, "not supported");
  }
  const uint8_t* getSelfIdPubKey() override { return self_id.pub_key; }
  void clearStats() override { }
  void applyTempRadioParams(float freq, float bw, uint8_t sf, uint8_t cr, int timeout_mins) override;

  float getTelemValue(uint8_t channel, uint8_t type);

protected:
  // current telemetry data queries
  float getVoltage(uint8_t channel) { return getTelemValue(channel, LPP_VOLTAGE); }
  float getCurrent(uint8_t channel) { return getTelemValue(channel, LPP_CURRENT); }
  float getPower(uint8_t channel) { return getTelemValue(channel, LPP_POWER); }
  float getTemperature(uint8_t channel) { return getTelemValue(channel, LPP_TEMPERATURE); }
  float getRelativeHumidity(uint8_t channel) { return getTelemValue(channel, LPP_RELATIVE_HUMIDITY); }
  float getBarometricPressure(uint8_t channel) { return getTelemValue(channel, LPP_BAROMETRIC_PRESSURE); }
  float getAltitude(uint8_t channel) { return getTelemValue(channel, LPP_ALTITUDE); }
  bool  getGPS(uint8_t channel, float& lat, float& lon, float& alt);

  // alerts
  enum AlertPriority { LOW_PRI_ALERT, HIGH_PRI_ALERT };

  struct Trigger {
    uint32_t timestamp;
    AlertPriority pri;
    uint32_t expected_acks[4];
    int8_t   curr_contact_idx;
    uint8_t  attempt;
    unsigned long send_expiry;
    char text[MAX_PACKET_PAYLOAD];

    Trigger() { text[0] = 0; }
    bool isTriggered() const { return text[0] != 0; }
  };
  void alertIf(bool condition, Trigger& t, AlertPriority pri, const char* text);

  virtual void onSensorDataRead() = 0;   // for app to implement
  virtual int querySeriesData(uint32_t start_secs_ago, uint32_t end_secs_ago, MinMaxAvg dest[], int max_num) = 0;  // for app to implement
  virtual bool handleCustomCommand(uint32_t sender_timestamp, char* command, char* reply) { return false; }

  // Mesh overrides
  float getAirtimeBudgetFactor() const override;
  bool allowPacketForward(const mesh::Packet* packet) override;
  int calcRxDelay(float score, uint32_t air_time) const override;
  uint32_t getRetransmitDelay(const mesh::Packet* packet) override;
  uint32_t getDirectRetransmitDelay(const mesh::Packet* packet) override;
  int getInterferenceThreshold() const override;
  int getAGCResetInterval() const override;
  void onAnonDataRecv(mesh::Packet* packet, const uint8_t* secret, const mesh::Identity& sender, uint8_t* data, size_t len) override;
  int searchPeersByHash(const uint8_t* hash) override;
  void getPeerSharedSecret(uint8_t* dest_secret, int peer_idx) override;
  void onPeerDataRecv(mesh::Packet* packet, uint8_t type, int sender_idx, const uint8_t* secret, uint8_t* data, size_t len) override;
  bool onPeerPathRecv(mesh::Packet* packet, int sender_idx, const uint8_t* secret, uint8_t* path, uint8_t path_len, uint8_t extra_type, uint8_t* extra, uint8_t extra_len) override;
  void onAckRecv(mesh::Packet* packet, uint32_t ack_crc) override;

private:
  FILESYSTEM* _fs;
  unsigned long next_local_advert, next_flood_advert;
  NodePrefs _prefs;
  CommonCLI _cli;
  uint8_t reply_data[MAX_PACKET_PAYLOAD];
  ContactInfo contacts[MAX_CONTACTS];
  int num_contacts;
  unsigned long dirty_contacts_expiry;
  CayenneLPP telemetry;
  uint32_t last_read_time;
  int matching_peer_indexes[MAX_SEARCH_RESULTS];
  int num_alert_tasks;
  Trigger* alert_tasks[MAX_CONCURRENT_ALERTS];
  unsigned long set_radio_at, revert_radio_at;
  float pending_freq;
  float pending_bw;
  uint8_t pending_sf;
  uint8_t pending_cr;

  void loadContacts();
  void saveContacts();
  uint8_t handleLoginReq(const mesh::Identity& sender, const uint8_t* secret, uint32_t sender_timestamp, const uint8_t* data);
  uint8_t handleRequest(uint8_t perms, uint32_t sender_timestamp, uint8_t req_type, uint8_t* payload, size_t payload_len);
  mesh::Packet* createSelfAdvert();
  ContactInfo* getContact(const uint8_t* pubkey, int key_len);
  ContactInfo* putContact(const mesh::Identity& id, uint8_t init_perms);
  bool applyContactPermissions(const uint8_t* pubkey, int key_len, uint8_t perms);

  void sendAlert(ContactInfo* c, Trigger* t);

};
