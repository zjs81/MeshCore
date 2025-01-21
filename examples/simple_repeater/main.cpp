#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#if defined(NRF52_PLATFORM)
  #include <InternalFileSystem.h>
#elif defined(ESP32)
  #include <SPIFFS.h>
#endif

#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/CustomSX1262Wrapper.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/IdentityStore.h>

/* ------------------------------ Config -------------------------------- */

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
  #defne LORA_TX_POWER  20
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
  #define  ADMIN_PASSWORD  "h^(kl@#)"
#endif

#if defined(HELTEC_LORA_V3)
  #include <helpers/HeltecV3Board.h>
  static HeltecV3Board board;
#elif defined(ARDUINO_XIAO_ESP32C3)
  #include <helpers/XiaoC3Board.h>
  #include <helpers/CustomSX1262Wrapper.h>
  #include <helpers/CustomSX1268Wrapper.h>
  static XiaoC3Board board;
#elif defined(SEEED_XIAO_S3)
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

#define CMD_GET_STATS      0x01
#define CMD_SET_CLOCK      0x02
#define CMD_SEND_ANNOUNCE  0x03
#define CMD_SET_CONFIG     0x04

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

class MyMesh : public mesh::Mesh {
  RadioLibWrapper* my_radio;
  float airtime_factor;
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
      case CMD_SET_CLOCK: {
        if (payload_len >= 5) {
          uint32_t curr_epoch_secs;
          memcpy(&curr_epoch_secs, &payload[1], 4);    // first param is current UNIX time

          if (curr_epoch_secs > now) {   // time can only go forward!!
            getRTCClock()->setCurrentTime(curr_epoch_secs);
            memcpy(&reply_data[4], "OK", 2);
          } else {
            memcpy(&reply_data[4], "ER", 2);
          }
          return 4 + 2;   //  reply_len
        }
        return 0;  // invalid request 
      }
      case CMD_SEND_ANNOUNCE: {
        // broadcast another self Advertisement
        sendSelfAdvertisement();

        memcpy(&reply_data[4], "OK", 2);
        return 4 + 2;  // reply_len
      }
      case CMD_SET_CONFIG: {
        if (payload_len >= 4 && payload_len < 32 && memcmp(&payload[1], "AF", 2) == 0) {
          payload[payload_len] = 0;  // make it a C string
          airtime_factor = atof((char *) &payload[3]);

          memcpy(&reply_data[4], "OK", 2);
          return 4 + 2;  // reply_len
        }        
        return 0;  // unknown config var
      }
    }
    // unknown command
    return 0;  // reply_len
  }

protected:
  float getAirtimeBudgetFactor() const override {
    return airtime_factor;
  }

  bool allowPacketForward(const mesh::Packet* packet) override {
    return true;   // Yes, allow packet to be forwarded
  }

  void onAnonDataRecv(mesh::Packet* packet, uint8_t type, const mesh::Identity& sender, uint8_t* data, size_t len) override {
    if (type == PAYLOAD_TYPE_ANON_REQ) {  // received an initial request by a possible admin client (unknown at this stage)
      uint32_t timestamp;
      memcpy(&timestamp, data, 4);

      if (memcmp(&data[4], ADMIN_PASSWORD, 8) == 0) {  // check for valid password
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
     if (type == PAYLOAD_TYPE_REQ) {  // request (from a Known admin client!)
      int i = matching_peer_indexes[sender_idx];

      if (i >= 0 && i < num_clients) {  // get from our known_clients table (sender SHOULD already be known in this context)
        auto client = &known_clients[i];

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
        }
      } else {
        MESH_DEBUG_PRINTLN("onPeerDataRecv: invalid peer idx: %d", i);
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
    airtime_factor = 1.0;    // one half
    num_clients = 0;
  }

  #define ADV_TYPE_NONE         0   // unknown
  #define ADV_TYPE_CHAT         1
  #define ADV_TYPE_REPEATER     2
  //FUTURE: 3..15

  #define ADV_LATLON_MASK       0x10
  #define ADV_BATTERY_MASK      0x20
  #define ADV_TEMPERATURE_MASK  0x40
  #define ADV_NAME_MASK         0x80

  void sendSelfAdvertisement() {
    uint8_t app_data[MAX_ADVERT_DATA_SIZE+32];
    app_data[0] = ADV_TYPE_REPEATER | ADV_NAME_MASK;
    int i = 1;
    int32_t lat = ADVERT_LAT * 1E6;
    int32_t lon = ADVERT_LON * 1E6;
    if (!(lat == 0 && lon == 0)) {
      app_data[0] |= ADV_LATLON_MASK;
      memcpy(&app_data[i], &lat, 4); i += 4;
      memcpy(&app_data[i], &lon, 4); i += 4;
    }
    strcpy((char *)&app_data[i], ADVERT_NAME);
    int app_data_len = i + strlen(ADVERT_NAME);
    if (app_data_len > MAX_ADVERT_DATA_SIZE) {
      app_data_len = MAX_ADVERT_DATA_SIZE;
      app_data[MAX_ADVERT_DATA_SIZE - 1] = 0;  // truncate the ADVERT_NAME
    }
 
    mesh::Packet* pkt = createAdvert(self_id, app_data, app_data_len);
    if (pkt) {
      sendFlood(pkt, 800);  // add slight delay
    } else {
      MESH_DEBUG_PRINTLN("ERROR: unable to create advertisement packet!");
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
MyMesh the_mesh(*new WRAPPER_CLASS(radio, board), *new ArduinoMillis(), fast_rng, *new VolatileRTCClock(), tables);

void halt() {
  while (1) ;
}

static char command[80];

void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();

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

#if defined(NRF52_PLATFORM)
  InternalFS.begin();
  IdentityStore store(InternalFS, "/identity");
#elif defined(ESP32)
  SPIFFS.begin(true);
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

  the_mesh.begin();

  // send out initial Advertisement to the mesh
  the_mesh.sendSelfAdvertisement();
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
    if (strcmp(command, "reboot") == 0) {
      board.reboot();  // doesn't return
    } else if (strcmp(command, "advert") == 0) {
      the_mesh.sendSelfAdvertisement();
    } else {
      Serial.print("   ERROR: unknown command: "); Serial.println(command);
      Serial.println("   (commands: reboot, advert)");
    }

    command[0] = 0;  // reset command buffer
  }

  the_mesh.loop();

  // TODO: periodically check for OLD/inactive entries in known_clients[], and evict
}
