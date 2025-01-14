#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>
#include <SPIFFS.h>

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
  #define LORA_BW     125
#endif
#ifndef LORA_SF
  #define LORA_SF     10
#endif
#ifndef LORA_CR
  #define LORA_CR      5
#endif

#define  ANNOUNCE_DATA   "repeater:v1"

#define  ADMIN_PASSWORD  "h^(kl@#)"

#if defined(HELTEC_LORA_V3)
  #include <helpers/HeltecV3Board.h>
  static HeltecV3Board board;
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
        auto adv = createAdvert(self_id, (const uint8_t *)ANNOUNCE_DATA, strlen(ANNOUNCE_DATA));
        if (adv) sendFlood(adv, 1500);   // send after slight delay

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
          return;  // FATAL: client table is full -OR- replay attack 
        }

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

  void onPeerDataRecv(mesh::Packet* packet, uint8_t type, int sender_idx, uint8_t* data, size_t len) override {
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
            mesh::Packet* path = createPathReturn(client->id, client->secret, packet->path, packet->path_len,
                                                  PAYLOAD_TYPE_RESPONSE, reply_data, reply_len);
            if (path) sendFlood(path);
          } else {
            mesh::Packet* reply = createDatagram(PAYLOAD_TYPE_RESPONSE, client->id, client->secret, reply_data, reply_len);
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

  void onPeerPathRecv(mesh::Packet* packet, int sender_idx, uint8_t* path, uint8_t path_len, uint8_t extra_type, uint8_t* extra, uint8_t extra_len) override {
    // TODO: prevent replay attacks
    int i = matching_peer_indexes[sender_idx];

    if (i >= 0 && i < num_clients) {  // get from our known_clients table (sender SHOULD already be known in this context)
      Serial.printf("PATH to client, path_len=%d\n", (uint32_t) path_len);
      auto client = &known_clients[i];
      memcpy(client->out_path, path, client->out_path_len = path_len);  // store a copy of path, for sendDirect()
    } else {
      MESH_DEBUG_PRINTLN("onPeerPathRecv: invalid peer idx: %d", i);
    }

    // NOTE: no reciprocal path send!!
  }

public:
  MyMesh(RadioLibWrapper& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : mesh::Mesh(radio, ms, rng, rtc, *new StaticPoolPacketManager(32), tables)
  {
    my_radio = &radio;
    airtime_factor = 5.0;   // 1/6th
    num_clients = 0;
  }

  void sendSelfAdvertisement() {
    mesh::Packet* pkt = createAdvert(self_id, (const uint8_t *)ANNOUNCE_DATA, strlen(ANNOUNCE_DATA));
    if (pkt) {
      sendFlood(pkt);
    } else {
      MESH_DEBUG_PRINTLN("ERROR: unable to create advertisement packet!");
    }
  }
};

#if defined(P_LORA_SCLK)
SPIClass spi;
CustomSX1262 radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, spi);
#else
CustomSX1262 radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY);
#endif
StdRNG fast_rng;
SimpleMeshTables tables;
MyMesh the_mesh(*new CustomSX1262Wrapper(radio, board), *new ArduinoMillis(), fast_rng, *new VolatileRTCClock(), tables);

void halt() {
  while (1) ;
}

void setup() {
  Serial.begin(115200);
  delay(5000);

  board.begin();

#ifdef SX126X_DIO3_TCXO_VOLTAGE
  float tcxo = SX126X_DIO3_TCXO_VOLTAGE;
#else
  float tcxo = 1.6f;
#endif

#if defined(P_LORA_SCLK)
  spi.begin(P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI);
  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22, 8, tcxo);
#else
  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22, 8, tcxo);
#endif
  if (status != RADIOLIB_ERR_NONE) {
    Serial.print("ERROR: radio init failed: ");
    Serial.println(status);
    halt();
  }

#ifdef SX126X_CURRENT_LIMIT
  radio.setCurrentLimit(SX126X_CURRENT_LIMIT);
#endif

#ifdef SX126X_DIO2_AS_RF_SWITCH
  radio.setDio2AsRfSwitch(SX126X_DIO2_AS_RF_SWITCH);
#endif

  SPIFFS.begin(true);
  IdentityStore store(SPIFFS, "/identity");
  if (!store.load("_main", the_mesh.self_id)) {
    the_mesh.self_id = mesh::LocalIdentity(the_mesh.getRNG());  // create new random identity
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Repeater ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  the_mesh.begin();

  // send out initial Advertisement to the mesh
  the_mesh.sendSelfAdvertisement();
}

void loop() {
  the_mesh.loop();

  // TODO: periodically check for OLD/inactive entries in known_clients[], and evict
}
