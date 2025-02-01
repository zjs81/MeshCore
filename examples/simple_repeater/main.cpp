#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#if defined(NRF52_PLATFORM)
  #include <InternalFileSystem.h>
#elif defined(ESP32)
  #include <SPIFFS.h>
#endif

#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/IdentityStore.h>
#include <helpers/AdvertDataHelpers.h>
#include <helpers/TxtDataHelpers.h>
#include <RTClib.h>

/* ------------------------------ Config -------------------------------- */

#define FIRMWARE_VER_TEXT   "v2 (build: 1 Feb 2025)"

#ifndef LORA_FREQ
  #define LORA_FREQ   915.0
#endif
#ifndef LORA_BW
  #define LORA_BW     250
#endif
#ifndef LORA_SF
  #define LORA_SF     10
#endif
#ifndef LORA_CR
  #define LORA_CR      5
#endif
#ifndef LORA_TX_POWER
  #define LORA_TX_POWER  20
#endif

#ifndef ADVERT_NAME
  #define  ADVERT_NAME   "repeater"
#endif
#ifndef ADVERT_LAT
  #define  ADVERT_LAT  0.0
#endif
#ifndef ADVERT_LON
  #define  ADVERT_LON  0.0
#endif

#ifndef ADMIN_PASSWORD
  #define  ADMIN_PASSWORD  "password"
#endif

#if defined(HELTEC_LORA_V3)
  #include <helpers/HeltecV3Board.h>
  #include <helpers/CustomSX1262Wrapper.h>
  static HeltecV3Board board;
#elif defined(ARDUINO_XIAO_ESP32C3)
  #include <helpers/XiaoC3Board.h>
  #include <helpers/CustomSX1262Wrapper.h>
  #include <helpers/CustomSX1268Wrapper.h>
  static XiaoC3Board board;
#elif defined(SEEED_XIAO_S3) || defined(LILYGO_T3S3)
  #include <helpers/ESP32Board.h>
  #include <helpers/CustomSX1262Wrapper.h>
  static ESP32Board board;
#elif defined(RAK_4631)
  #include <helpers/RAK4631Board.h>
  #include <helpers/CustomSX1262Wrapper.h>
  static RAK4631Board board;
#else
  #error "need to provide a 'board' object"
#endif

/* ------------------------------ Code -------------------------------- */

// Believe it or not, this std C function is busted on some platforms!
static uint32_t _atoi(const char* sp) {
  uint32_t n = 0;
  while (*sp && *sp >= '0' && *sp <= '9') {
    n *= 10;
    n += (*sp++ - '0');
  }
  return n;
}

#define CMD_GET_STATS      0x01

struct RepeaterStats {
  uint16_t batt_milli_volts;
  uint16_t curr_tx_queue_len;
  uint16_t curr_free_queue_len;
  int16_t  last_rssi;
  uint32_t n_packets_recv;
  uint32_t n_packets_sent;
  uint32_t total_air_time_secs;
  uint32_t total_up_time_secs;
  uint32_t n_sent_flood, n_sent_direct;
  uint32_t n_recv_flood, n_recv_direct;
  uint32_t n_full_events;
};

struct ClientInfo {
  mesh::Identity id;
  uint32_t last_timestamp;
  uint8_t secret[PUB_KEY_SIZE];
  int out_path_len;
  uint8_t out_path[MAX_PATH_SIZE];
};

#define MAX_CLIENTS   4

// NOTE: need to space the ACK and the reply text apart (in CLI)
#define CLI_REPLY_DELAY_MILLIS  1500

struct NodePrefs {  // persisted to file
  float airtime_factor;
  char node_name[32];
  double node_lat, node_lon;
  char password[16];
  float freq;
  uint8_t tx_power_dbm;
  uint8_t unused[3];
};

class MyMesh : public mesh::Mesh {
  RadioLibWrapper* my_radio;
  FILESYSTEM* _fs;
  NodePrefs _prefs;
  uint8_t reply_data[MAX_PACKET_PAYLOAD];
  int num_clients;
  ClientInfo known_clients[MAX_CLIENTS];

  ClientInfo* putClient(const mesh::Identity& id) {
    for (int i = 0; i < num_clients; i++) {
      if (id.matches(known_clients[i].id)) return &known_clients[i];  // already known
    }
    if (num_clients < MAX_CLIENTS) {
      auto newClient = &known_clients[num_clients++];
      newClient->id = id;
      newClient->out_path_len = -1;  // initially out_path is unknown
      newClient->last_timestamp = 0;
      self_id.calcSharedSecret(newClient->secret, id);   // calc ECDH shared secret
      return newClient;
    }
    return NULL;  // table is full
  }

  int handleRequest(ClientInfo* sender, uint8_t* payload, size_t payload_len) { 
    uint32_t now = getRTCClock()->getCurrentTime();
    memcpy(reply_data, &now, 4);   // response packets always prefixed with timestamp

    switch (payload[0]) {
      case CMD_GET_STATS: {
        uint32_t max_age_secs;
        if (payload_len >= 5) {
          memcpy(&max_age_secs, &payload[1], 4);    // first param in request pkt
        } else {
          max_age_secs = 12*60*60;   // default, 12 hours
        }

        RepeaterStats stats;
        stats.batt_milli_volts = board.getBattMilliVolts();
        stats.curr_tx_queue_len = _mgr->getOutboundCount();
        stats.curr_free_queue_len = _mgr->getFreeCount();
        stats.last_rssi = (int16_t) my_radio->getLastRSSI();
        stats.n_packets_recv = my_radio->getPacketsRecv();
        stats.n_packets_sent = my_radio->getPacketsSent();
        stats.total_air_time_secs = getTotalAirTime() / 1000;
        stats.total_up_time_secs = _ms->getMillis() / 1000;
        stats.n_sent_flood = getNumSentFlood();
        stats.n_sent_direct = getNumSentDirect();
        stats.n_recv_flood = getNumRecvFlood();
        stats.n_recv_direct = getNumRecvDirect();
        stats.n_full_events = getNumFullEvents();

        memcpy(&reply_data[4], &stats, sizeof(stats));

        return 4 + sizeof(stats);  //  reply_len
      }
    }
    // unknown command
    return 0;  // reply_len
  }

protected:
  float getAirtimeBudgetFactor() const override {
    return _prefs.airtime_factor;
  }

  bool allowPacketForward(const mesh::Packet* packet) override {
    return true;   // Yes, allow packet to be forwarded
  }

  void onAnonDataRecv(mesh::Packet* packet, uint8_t type, const mesh::Identity& sender, uint8_t* data, size_t len) override {
    if (type == PAYLOAD_TYPE_ANON_REQ) {  // received an initial request by a possible admin client (unknown at this stage)
      uint32_t timestamp;
      memcpy(&timestamp, data, 4);

      if (memcmp(&data[4], _prefs.password, strlen(_prefs.password)) == 0) {  // check for valid password
        auto client = putClient(sender);  // add to known clients (if not already known)
        if (client == NULL || timestamp <= client->last_timestamp) {
          MESH_DEBUG_PRINTLN("Client table full, or replay attack!");
          return;  // FATAL: client table is full -OR- replay attack 
        }

        MESH_DEBUG_PRINTLN("Login success!");
        client->last_timestamp = timestamp;

        uint32_t now = getRTCClock()->getCurrentTime();
        memcpy(reply_data, &now, 4);   // response packets always prefixed with timestamp
        memcpy(&reply_data[4], "OK", 2);

        if (packet->isRouteFlood()) {
          // let this sender know path TO here, so they can use sendDirect(), and ALSO encode the response
          mesh::Packet* path = createPathReturn(sender, client->secret, packet->path, packet->path_len,
                                                PAYLOAD_TYPE_RESPONSE, reply_data, 4 + 2);
          if (path) sendFlood(path);
        } else {
          mesh::Packet* reply = createDatagram(PAYLOAD_TYPE_RESPONSE, sender, client->secret, reply_data, 4 + 2);
          if (reply) {
            if (client->out_path_len >= 0) {  // we have an out_path, so send DIRECT
              sendDirect(reply, client->out_path, client->out_path_len);
            } else {
              sendFlood(reply);
            }
          }
        }
      } else {
        data[4+8] = 0;  // ensure null terminator
        MESH_DEBUG_PRINTLN("Incorrect password: %s", &data[4]);
      }
    }
  }

  int  matching_peer_indexes[MAX_CLIENTS];

  int searchPeersByHash(const uint8_t* hash) override {
    int n = 0;
    for (int i = 0; i < num_clients; i++) {
      if (known_clients[i].id.isHashMatch(hash)) {
        matching_peer_indexes[n++] = i;  // store the INDEXES of matching contacts (for subsequent 'peer' methods)
      }
    }
    return n;
  }

  void getPeerSharedSecret(uint8_t* dest_secret, int peer_idx) override {
    int i = matching_peer_indexes[peer_idx];
    if (i >= 0 && i < num_clients) {
      // lookup pre-calculated shared_secret
      memcpy(dest_secret, known_clients[i].secret, PUB_KEY_SIZE);
    } else {
      MESH_DEBUG_PRINTLN("getPeerSharedSecret: Invalid peer idx: %d", i);
    }
  }

  void onPeerDataRecv(mesh::Packet* packet, uint8_t type, int sender_idx, const uint8_t* secret, uint8_t* data, size_t len) override {
    int i = matching_peer_indexes[sender_idx];
    if (i < 0 || i >= num_clients) {  // get from our known_clients table (sender SHOULD already be known in this context)
      MESH_DEBUG_PRINTLN("onPeerDataRecv: invalid peer idx: %d", i);
      return;
    }
    auto client = &known_clients[i];
    if (type == PAYLOAD_TYPE_REQ) {  // request (from a Known admin client!)
      uint32_t timestamp;
      memcpy(&timestamp, data, 4);

      if (timestamp > client->last_timestamp) {  // prevent replay attacks 
        int reply_len = handleRequest(client, &data[4], len - 4);
        if (reply_len == 0) return;  // invalid command

        client->last_timestamp = timestamp;

        if (packet->isRouteFlood()) {
          // let this sender know path TO here, so they can use sendDirect(), and ALSO encode the response
          mesh::Packet* path = createPathReturn(client->id, secret, packet->path, packet->path_len,
                                                PAYLOAD_TYPE_RESPONSE, reply_data, reply_len);
          if (path) sendFlood(path);
        } else {
          mesh::Packet* reply = createDatagram(PAYLOAD_TYPE_RESPONSE, client->id, secret, reply_data, reply_len);
          if (reply) {
            if (client->out_path_len >= 0) {  // we have an out_path, so send DIRECT
              sendDirect(reply, client->out_path, client->out_path_len);
            } else {
              sendFlood(reply);
            }
          }
        }
      } else {
        MESH_DEBUG_PRINTLN("onPeerDataRecv: possible replay attack detected");
      }
    } else if (type == PAYLOAD_TYPE_TXT_MSG && len > 5) {   // a CLI command
      uint32_t sender_timestamp;
      memcpy(&sender_timestamp, data, 4);  // timestamp (by sender's RTC clock - which could be wrong)
      uint flags = (data[4] >> 2);   // message attempt number, and other flags

      if (!(flags == TXT_TYPE_PLAIN || flags == TXT_TYPE_CLI_DATA)) {
        MESH_DEBUG_PRINTLN("onPeerDataRecv: unsupported text type received: flags=%02x", (uint32_t)flags);
      } else if (sender_timestamp > client->last_timestamp) {  // prevent replay attacks 
        client->last_timestamp = sender_timestamp;

        // len can be > original length, but 'text' will be padded with zeroes
        data[len] = 0; // need to make a C string again, with null terminator

        uint32_t ack_hash;    // calc truncated hash of the message timestamp + text + sender pub_key, to prove to sender that we got it
        mesh::Utils::sha256((uint8_t *) &ack_hash, 4, data, 5 + strlen((char *)&data[5]), client->id.pub_key, PUB_KEY_SIZE);

        mesh::Packet* ack = createAck(ack_hash);
        if (ack) {
          if (client->out_path_len < 0) {
            sendFlood(ack);
          } else {
            sendDirect(ack, client->out_path, client->out_path_len);
          }
        }

        uint8_t temp[166];
        handleCommand(sender_timestamp, (const char *) &data[5], (char *) &temp[5]);
        int text_len = strlen((char *) &temp[5]);
        if (text_len > 0) {
          uint32_t timestamp = getRTCClock()->getCurrentTime();
          if (timestamp == sender_timestamp) {
            // WORKAROUND: the two timestamps need to be different, in the CLI view
            timestamp++;
          }
          memcpy(temp, &timestamp, 4);   // mostly an extra blob to help make packet_hash unique
          temp[4] = (TXT_TYPE_PLAIN << 2);   // TODO: change this to TXT_TYPE_CLI_DATA soon

          // calc expected ACK reply
          //mesh::Utils::sha256((uint8_t *)&expected_ack_crc, 4, temp, 5 + text_len, self_id.pub_key, PUB_KEY_SIZE);

          auto reply = createDatagram(PAYLOAD_TYPE_TXT_MSG, client->id, secret, temp, 5 + text_len);
          if (reply) {
            if (client->out_path_len < 0) {
              sendFlood(reply, CLI_REPLY_DELAY_MILLIS);
            } else {
              sendDirect(reply, client->out_path, client->out_path_len, CLI_REPLY_DELAY_MILLIS);
            }
          }
        }
      } else {
        MESH_DEBUG_PRINTLN("onPeerDataRecv: possible replay attack detected");
      }
    }
  }

  bool onPeerPathRecv(mesh::Packet* packet, int sender_idx, const uint8_t* secret, uint8_t* path, uint8_t path_len, uint8_t extra_type, uint8_t* extra, uint8_t extra_len) override {
    // TODO: prevent replay attacks
    int i = matching_peer_indexes[sender_idx];

    if (i >= 0 && i < num_clients) {  // get from our known_clients table (sender SHOULD already be known in this context)
      MESH_DEBUG_PRINTLN("PATH to client, path_len=%d", (uint32_t) path_len);
      auto client = &known_clients[i];
      memcpy(client->out_path, path, client->out_path_len = path_len);  // store a copy of path, for sendDirect()
    } else {
      MESH_DEBUG_PRINTLN("onPeerPathRecv: invalid peer idx: %d", i);
    }

    // NOTE: no reciprocal path send!!
    return false;
  }

public:
  MyMesh(RadioLibWrapper& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : mesh::Mesh(radio, ms, rng, rtc, *new StaticPoolPacketManager(32), tables)
  {
    my_radio = &radio;
    num_clients = 0;

    // defaults
    _prefs.airtime_factor = 1.0;    // one half
    strncpy(_prefs.node_name, ADVERT_NAME, sizeof(_prefs.node_name)-1);
    _prefs.node_name[sizeof(_prefs.node_name)-1] = 0;  // truncate if necessary
    _prefs.node_lat = ADVERT_LAT;
    _prefs.node_lon = ADVERT_LON;
    strncpy(_prefs.password, ADMIN_PASSWORD, sizeof(_prefs.password)-1);
    _prefs.password[sizeof(_prefs.password)-1] = 0;  // truncate if necessary
    _prefs.freq = LORA_FREQ;
    _prefs.tx_power_dbm = LORA_TX_POWER;
    memset(_prefs.unused, 0, sizeof(_prefs.unused));
  }

  float getFreqPref() const { return _prefs.freq; }
  uint8_t getTxPowerPref() const { return _prefs.tx_power_dbm; }

  void begin(FILESYSTEM* fs) {
    mesh::Mesh::begin();
    _fs = fs;
    // load persisted prefs
    if (_fs->exists("/node_prefs")) {
      File file = _fs->open("/node_prefs");
      if (file) {
        file.read((uint8_t *) &_prefs, sizeof(_prefs));
        file.close();
      }
    }
  }

  void savePrefs() {
#if defined(NRF52_PLATFORM)
    File file = _fs->open("/node_prefs", FILE_O_WRITE);
    if (file) { file.seek(0); file.truncate(); }
#else
    File file = _fs->open("/node_prefs", "w", true);
#endif
    if (file) {
      file.write((const uint8_t *)&_prefs, sizeof(_prefs));
      file.close();
    }
  }

  void sendSelfAdvertisement(int delay_millis) {
    uint8_t app_data[MAX_ADVERT_DATA_SIZE];
    uint8_t app_data_len;
    {
      AdvertDataBuilder builder(ADV_TYPE_REPEATER, _prefs.node_name, _prefs.node_lat, _prefs.node_lon);
      app_data_len = builder.encodeTo(app_data);
    }

    mesh::Packet* pkt = createAdvert(self_id, app_data, app_data_len);
    if (pkt) {
      sendFlood(pkt, delay_millis);
    } else {
      MESH_DEBUG_PRINTLN("ERROR: unable to create advertisement packet!");
    }
  }

  void handleCommand(uint32_t sender_timestamp, const char* command, char reply[]) {
    while (*command == ' ') command++;   // skip leading spaces

    if (memcmp(command, "reboot", 6) == 0) {
      board.reboot();  // doesn't return
    } else if (memcmp(command, "advert", 6) == 0) {
      sendSelfAdvertisement(400);
      strcpy(reply, "OK - Advert sent");
    } else if (memcmp(command, "clock sync", 10) == 0) {
      uint32_t curr = getRTCClock()->getCurrentTime();
      if (sender_timestamp > curr) {
        getRTCClock()->setCurrentTime(sender_timestamp + 1);
        strcpy(reply, "OK - clock set");
      } else {
        strcpy(reply, "ERR: clock cannot go backwards");
      }
    } else if (memcmp(command, "clock", 5) == 0) {
      uint32_t now = getRTCClock()->getCurrentTime();
      DateTime dt = DateTime(now);
      sprintf(reply, "%02d:%02d - %d/%d/%d UTC", dt.hour(), dt.minute(), dt.day(), dt.month(), dt.year());
    } else if (memcmp(command, "time ", 5) == 0) {  // set time (to epoch seconds)
      uint32_t secs = _atoi(&command[5]);
      uint32_t curr = getRTCClock()->getCurrentTime();
      if (secs > curr) {
        getRTCClock()->setCurrentTime(secs);
        strcpy(reply, "(OK - clock set!)");
      } else {
        strcpy(reply, "(ERR: clock cannot go backwards)");
      }
    } else if (memcmp(command, "password ", 9) == 0) {
      // change admin password
      strncpy(_prefs.password, &command[9], sizeof(_prefs.password)-1);
      _prefs.password[sizeof(_prefs.password)-1] = 0;  // truncate if necesary
      savePrefs();
      sprintf(reply, "password now: %s", _prefs.password);   // echo back just to let admin know for sure!!
    } else if (memcmp(command, "set ", 4) == 0) {
      const char* config = &command[4];
      if (memcmp(config, "af ", 3) == 0) {
        _prefs.airtime_factor = atof(&config[3]);
        savePrefs();
        strcpy(reply, "OK");
      } else if (memcmp(config, "name ", 5) == 0) {
        strncpy(_prefs.node_name, &config[5], sizeof(_prefs.node_name)-1);
        _prefs.node_name[sizeof(_prefs.node_name)-1] = 0;  // truncate if nec
        savePrefs();
        strcpy(reply, "OK");
      } else if (memcmp(config, "lat ", 4) == 0) {
        _prefs.node_lat = atof(&config[4]);
        savePrefs();
        strcpy(reply, "OK");
      } else if (memcmp(config, "lon ", 4) == 0) {
        _prefs.node_lon = atof(&config[4]);
        savePrefs();
        strcpy(reply, "OK");
      } else if (memcmp(config, "tx ", 3) == 0) {
        _prefs.tx_power_dbm = atoi(&config[3]);
        savePrefs();
        strcpy(reply, "OK - reboot to apply");
      } else if (sender_timestamp == 0 && memcmp(config, "freq ", 5) == 0) {
        _prefs.freq = atof(&config[5]);
        savePrefs();
        strcpy(reply, "OK - reboot to apply");
      } else {
        sprintf(reply, "unknown config: %s", config);
      }
    } else if (memcmp(command, "ver", 3) == 0) {
      strcpy(reply, FIRMWARE_VER_TEXT);
    } else {
      sprintf(reply, "Unknown: %s (commands: reboot, advert, clock, set, ver)", command);
    }
  }
};

#if defined(NRF52_PLATFORM)
RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, SPI);
#elif defined(P_LORA_SCLK)
SPIClass spi;
RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, spi);
#else
RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY);
#endif
StdRNG fast_rng;
SimpleMeshTables tables;

#ifdef ESP32
ESP32RTCClock rtc_clock;
#else
VolatileRTCClock rtc_clock; 
#endif

MyMesh the_mesh(*new WRAPPER_CLASS(radio, board), *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void halt() {
  while (1) ;
}

static char command[80];

void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();
#ifdef ESP32
  rtc_clock.begin();
#endif

#ifdef SX126X_DIO3_TCXO_VOLTAGE
  float tcxo = SX126X_DIO3_TCXO_VOLTAGE;
#else
  float tcxo = 1.6f;
#endif

#if defined(NRF52_PLATFORM)
  SPI.setPins(P_LORA_MISO, P_LORA_SCLK, P_LORA_MOSI);
  SPI.begin();
#elif defined(P_LORA_SCLK)
  spi.begin(P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI);
#endif
  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, LORA_TX_POWER, 8, tcxo);
  if (status != RADIOLIB_ERR_NONE) {
    delay(5000);
    Serial.print("ERROR: radio init failed: ");
    Serial.println(status);
    halt();
  }

  radio.setCRC(0);

#ifdef SX126X_CURRENT_LIMIT
  radio.setCurrentLimit(SX126X_CURRENT_LIMIT);
#endif

#ifdef SX126X_DIO2_AS_RF_SWITCH
  radio.setDio2AsRfSwitch(SX126X_DIO2_AS_RF_SWITCH);
#endif

  FILESYSTEM* fs;
#if defined(NRF52_PLATFORM)
  InternalFS.begin();
  fs = &InternalFS;
  IdentityStore store(InternalFS, "/identity");
#elif defined(ESP32)
  SPIFFS.begin(true);
  fs = &SPIFFS;
  IdentityStore store(SPIFFS, "/identity");
#else
  #error "need to define filesystem"
#endif
  if (!store.load("_main", the_mesh.self_id)) {
    the_mesh.self_id = mesh::LocalIdentity(the_mesh.getRNG());  // create new random identity
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Repeater ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  command[0] = 0;

  the_mesh.begin(fs);

  if (LORA_FREQ != the_mesh.getFreqPref()) {
    radio.setFrequency(the_mesh.getFreqPref());
  }
  if (LORA_TX_POWER != the_mesh.getTxPowerPref()) {
    radio.setOutputPower(the_mesh.getTxPowerPref());
  }

  // send out initial Advertisement to the mesh
  the_mesh.sendSelfAdvertisement(2000);
}

void loop() {
  int len = strlen(command);
  while (Serial.available() && len < sizeof(command)-1) {
    char c = Serial.read();
    if (c != '\n') { 
      command[len++] = c;
      command[len] = 0;
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

    command[0] = 0;  // reset command buffer
  }

  the_mesh.loop();

  // TODO: periodically check for OLD/inactive entries in known_clients[], and evict
}
