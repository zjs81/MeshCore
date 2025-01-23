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
#include <RTClib.h>

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
  #define LORA_TX_POWER  20
#endif

#ifndef ADVERT_NAME
  #define  ADVERT_NAME   "Test BBS"
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

#ifndef MAX_CLIENTS 
 #define MAX_CLIENTS           32
#endif

#ifndef MAX_UNSYNCED_POSTS
  #define MAX_UNSYNCED_POSTS    16
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

struct ClientInfo {
  mesh::Identity id;
  uint32_t last_timestamp;  // by THEIR clock
  uint32_t last_activity;   // by OUR clock
  uint32_t sync_since;  // sync messages SINCE this timestamp (by OUR clock)
  uint32_t pending_ack;
  uint32_t push_post_timestamp;
  unsigned long ack_timeout;
  bool     is_admin;
  uint8_t  push_failures;
  uint8_t  secret[PUB_KEY_SIZE];
  int      out_path_len;
  uint8_t  out_path[MAX_PATH_SIZE];
};

#define MAX_POST_TEXT_LEN    (160-9)

struct PostInfo {
  mesh::Identity author;
  uint32_t post_timestamp;   // by OUR clock
  char text[MAX_POST_TEXT_LEN+1];
};

#define REPLY_DELAY_MILLIS         1500
#define PUSH_NOTIFY_DELAY_MILLIS   1000
#define SYNC_PUSH_INTERVAL         1000

#define PUSH_ACK_TIMEOUT_FLOOD    12000
#define PUSH_TIMEOUT_BASE          4000
#define PUSH_ACK_TIMEOUT_FACTOR    2000

class MyMesh : public mesh::Mesh {
  RadioLibWrapper* my_radio;
  float airtime_factor;
  uint8_t reply_data[MAX_PACKET_PAYLOAD];
  int num_clients;
  ClientInfo known_clients[MAX_CLIENTS];
  unsigned long next_push;
  int next_client_idx;  // for round-robin polling
  int next_post_idx;
  PostInfo posts[MAX_UNSYNCED_POSTS];   // cyclic queue

  ClientInfo* putClient(const mesh::Identity& id) {
    for (int i = 0; i < num_clients; i++) {
      if (id.matches(known_clients[i].id)) return &known_clients[i];  // already known
    }
    ClientInfo* newClient;
    if (num_clients < MAX_CLIENTS) {
      newClient = &known_clients[num_clients++];
    } else {    // table is currently full
      // evict least active client
      uint32_t oldest_timestamp = 0xFFFFFFFF;
      newClient = &known_clients[0];
      for (int i = 0; i < num_clients; i++) {
        auto c = &known_clients[i];
        if (c->last_activity < oldest_timestamp) {
          oldest_timestamp = c->last_activity;
          newClient = c;
        }
      }
    }
    newClient->id = id;
    newClient->out_path_len = -1;  // initially out_path is unknown
    newClient->last_timestamp = 0;
    self_id.calcSharedSecret(newClient->secret, id);   // calc ECDH shared secret
    return newClient;
  }

  void  evict(ClientInfo* client) {
    client->last_activity = 0;  // this slot will now be re-used (will be oldest)
    memset(client->id.pub_key, 0, sizeof(client->id.pub_key));
    memset(client->secret, 0, sizeof(client->secret));
    client->pending_ack = 0;
  }

  void addPost(ClientInfo* client, const char* postData, char reply[]) {
    // TODO: suggested postData format: <title>/<descrption>
    posts[next_post_idx].author = client->id;    // add to cyclic queue
    strncpy(posts[next_post_idx].text, postData, MAX_POST_TEXT_LEN);
    posts[next_post_idx].text[MAX_POST_TEXT_LEN] = 0;

    posts[next_post_idx].post_timestamp = getRTCClock()->getCurrentTime();
    // TODO:  only post at maximum of ONE PER SECOND, so that post_timestamps are UNIQUE!!
    next_post_idx = (next_post_idx + 1) % MAX_UNSYNCED_POSTS;

    strcpy(reply, "[Posted]");
    next_push = futureMillis(PUSH_NOTIFY_DELAY_MILLIS);
  }

  void handleCommand(ClientInfo* client, const char* command, char reply[]) {
    if (*command == '+') {
      addPost(client, &command[1], reply);
    } else {
      strcpy(reply, "?");       // unknown command
    }
  }

  void pushPostToClient(ClientInfo* client, PostInfo& post) {
    int len = 0;
    memcpy(&reply_data[len], &post.post_timestamp, 4); len += 4;   // this is a PAST timestamp... but should be accepted by client
    reply_data[len++] = 0;  // plain text

    // encode prefix of post.author.pub_key (in hex)
    mesh::Utils::toHex((char *) &reply_data[len], post.author.pub_key, 4); len += 8;   // just first 4 bytes (8 hex chars)
    reply_data[len++] = ':';

    int text_len = strlen(post.text);
    memcpy(&reply_data[len], post.text, text_len); len += text_len;

    // calc expected ACK reply
    mesh::Utils::sha256((uint8_t *)&client->pending_ack, 4, reply_data, len, self_id.pub_key, PUB_KEY_SIZE);
    client->push_post_timestamp = post.post_timestamp;

    auto reply = createDatagram(PAYLOAD_TYPE_TXT_MSG, client->id, client->secret, reply_data, len);
    if (reply) {
      if (client->out_path_len < 0) {
        sendFlood(reply);
        client->ack_timeout = futureMillis(PUSH_ACK_TIMEOUT_FLOOD);
      } else {
        sendDirect(reply, client->out_path, client->out_path_len);
        client->ack_timeout = futureMillis(PUSH_TIMEOUT_BASE + PUSH_ACK_TIMEOUT_FACTOR * (client->out_path_len + 1));
      }
    } else {
      client->pending_ack = 0;
      MESH_DEBUG_PRINTLN("Unable to push post to client");
    }
  }

  bool processAck(const uint8_t *data) {
    for (int i = 0; i < num_clients; i++) {
      auto client = &known_clients[i];
      if (client->pending_ack && memcmp(data, &client->pending_ack, 4) == 0) {     // got an ACK from Client!
        client->pending_ack = 0;    // clear this, so next push can happen
        client->push_failures = 0;
        client->sync_since = client->push_post_timestamp;   // advance Client's SINCE timestamp, to sync next post
        return true;
      }
    }
    return false;
  }

protected:
  float getAirtimeBudgetFactor() const override {
    return airtime_factor;
  }

#if ROOM_IS_ALSO_REPEATER
  bool allowPacketForward(const mesh::Packet* packet) override {
    return true;   // Yes, allow packet to be forwarded
  }
#endif

  void onAnonDataRecv(mesh::Packet* packet, uint8_t type, const mesh::Identity& sender, uint8_t* data, size_t len) override {
    if (type == PAYLOAD_TYPE_ANON_REQ) {  // received an initial request by a possible admin client (unknown at this stage)
      uint32_t sender_timestamp, sender_sync_since;
      memcpy(&sender_timestamp, data, 4);
      memcpy(&sender_sync_since, &data[4], 4);  // sender's "sync messags SINCE x" timestamp

      bool is_admin;
      if (memcmp(&data[8], ADMIN_PASSWORD, strlen(ADMIN_PASSWORD)) == 0) {  // check for valid admin password
        is_admin = true;
      } else {
        is_admin = false;
    #ifdef ROOM_PASSWORD
        if (memcmp(&data[8], ROOM_PASSWORD, strlen(ROOM_PASSWORD)) != 0) {  // check the room/public password
          MESH_DEBUG_PRINTLN("Incorrect room password");
          return;   // no response. Client will timeout
        }
    #endif
      }

      auto client = putClient(sender);  // add to known clients (if not already known)
      if (sender_timestamp <= client->last_timestamp) {
        MESH_DEBUG_PRINTLN("possible replay attack!");
        return;
      }

      MESH_DEBUG_PRINTLN("Login success!");
      client->is_admin = is_admin;
      client->last_timestamp = sender_timestamp;
      client->sync_since = sender_sync_since;
      client->pending_ack = 0;
      client->push_failures = 0;

      uint32_t now = getRTCClock()->getCurrentTime();
      client->last_activity = now;

      memcpy(reply_data, &now, 4);   // response packets always prefixed with timestamp
      // TODO: maybe reply with count of messages waiting to be synced for THIS client?
      memset(&reply_data[4], 0, 4);  // FUTURE: reserve 4 bytes 
      memcpy(&reply_data[8], "OK", 2);

      if (packet->isRouteFlood()) {
        // let this sender know path TO here, so they can use sendDirect(), and ALSO encode the response
        mesh::Packet* path = createPathReturn(sender, client->secret, packet->path, packet->path_len,
                                              PAYLOAD_TYPE_RESPONSE, reply_data, 8 + 2);
        if (path) sendFlood(path);
      } else {
        mesh::Packet* reply = createDatagram(PAYLOAD_TYPE_RESPONSE, sender, client->secret, reply_data, 8 + 2);
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
    if (type == PAYLOAD_TYPE_TXT_MSG && len > 5) {   // a CLI command
      uint32_t sender_timestamp;
      memcpy(&sender_timestamp, data, 4);  // timestamp (by sender's RTC clock - which could be wrong)
      uint flags = data[4];   // message attempt number, and other flags

      if (flags != 0) {
        MESH_DEBUG_PRINTLN("onPeerDataRecv: unsupported command type received: flags=%02x", (uint32_t)flags);
      } else if (sender_timestamp > client->last_timestamp) {  // prevent replay attacks 
        client->last_timestamp = sender_timestamp;

        uint32_t now = getRTCClock()->getCurrentTime();
        client->last_activity = now;

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
        if (client->is_admin) {
          if (!handleAdminCommand(sender_timestamp, (const char *) &data[5], (char *) &temp[5])) {
            handleCommand(client, (const char *) &data[5], (char *) &temp[5]);
          }
        } else {
          handleCommand(client, (const char *) &data[5], (char *) &temp[5]);
        }

        int text_len = strlen((char *) &temp[5]);
        if (text_len > 0) {
          if (now == sender_timestamp) {
            // WORKAROUND: the two timestamps need to be different, in the CLI view
            now++;
          }
          memcpy(temp, &now, 4);   // mostly an extra blob to help make packet_hash unique
          temp[4] = 0;  // attempt and flags

          // calc expected ACK reply
          //mesh::Utils::sha256((uint8_t *)&expected_ack_crc, 4, temp, 5 + text_len, self_id.pub_key, PUB_KEY_SIZE);

          auto reply = createDatagram(PAYLOAD_TYPE_TXT_MSG, client->id, secret, temp, 5 + text_len);
          if (reply) {
            if (client->out_path_len < 0) {
              sendFlood(reply, REPLY_DELAY_MILLIS);
            } else {
              sendDirect(reply, client->out_path, client->out_path_len, REPLY_DELAY_MILLIS);
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

    if (extra_type == PAYLOAD_TYPE_ACK && extra_len >= 4) {
      // also got an encoded ACK!
      processAck(extra);
    }

    // NOTE: no reciprocal path send!!
    return false;
  }

  void onAckRecv(mesh::Packet* packet, uint32_t ack_crc) override {
    if (processAck((uint8_t *)&ack_crc)) {
      packet->markDoNotRetransmit();   // ACK was for this node, so don't retransmit
    }
  }

public:
  MyMesh(RadioLibWrapper& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : mesh::Mesh(radio, ms, rng, rtc, *new StaticPoolPacketManager(32), tables)
  {
    my_radio = &radio;
    airtime_factor = 1.0;    // one half
    num_clients = 0;
    next_post_idx = 0;
    next_client_idx = 0;
    next_push = 0;
    memset(posts, 0, sizeof(posts));
  }

  #define ADV_TYPE_NONE         0   // unknown
  #define ADV_TYPE_CHAT         1
  #define ADV_TYPE_REPEATER     2
  #define ADV_TYPE_ROOM         3   // New kid on the block!
  //FUTURE: 4..15

  #define ADV_LATLON_MASK       0x10
  #define ADV_BATTERY_MASK      0x20
  #define ADV_TEMPERATURE_MASK  0x40
  #define ADV_NAME_MASK         0x80

  void sendSelfAdvertisement() {
    uint8_t app_data[MAX_ADVERT_DATA_SIZE+32];
    app_data[0] = ADV_TYPE_ROOM | ADV_NAME_MASK;
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
      sendFlood(pkt, 1200);  // add slight delay
    } else {
      MESH_DEBUG_PRINTLN("ERROR: unable to create advertisement packet!");
    }
  }

  bool handleAdminCommand(uint32_t sender_timestamp, const char* command, char reply[]) {
    while (*command == ' ') command++;   // skip leading spaces

    if (memcmp(command, "reboot", 6) == 0) {
      board.reboot();  // doesn't return
    } else if (memcmp(command, "advert", 6) == 0) {
      sendSelfAdvertisement();
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
    } else if (memcmp(command, "set ", 4) == 0) {
      if (memcmp(&command[4], "AF", 2) == 0 || memcmp(&command[4], "af=", 2) == 0) {
        airtime_factor = atof(&command[7]);
        strcpy(reply, "OK");
      } else {
        sprintf(reply, "unknown config: %s", &command[4]);
      }
    } else {
      // unknown command
      reply[0] = 0;
      return false;
    }
    return true;
  }

  void loop() {
    mesh::Mesh::loop();

    if (millisHasNowPassed(next_push) && num_clients > 0) {
      // check for ACK timeouts
      for (int i = 0; i < num_clients; i++) {
        auto c = &known_clients[i];
        if (c->pending_ack && millisHasNowPassed(c->ack_timeout)) {
          c->push_failures++;
          c->pending_ack = 0;   // reset  (TODO: keep prev expected_ack's in a list, incase they arrive LATER, after we retry)

          if (c->push_failures >= 3) {
            evict(c);
          }
        }
      }
      // check next Round-Robin client, and sync next new post
      auto client = &known_clients[next_client_idx];
      if (client->pending_ack == 0 && client->last_activity != 0) {  // not already waiting for ACK, AND not evicted
        for (int k = 0, idx = next_post_idx; k < MAX_UNSYNCED_POSTS; k++) {
          if (posts[idx].post_timestamp > client->sync_since   // is new post for this Client?
            && !posts[idx].author.matches(client->id)) {    // don't push posts to the author
            // push this post to Client, then wait for ACK
            pushPostToClient(client, posts[idx]);
            break;
          }
          idx = (idx + 1) % MAX_UNSYNCED_POSTS;   // wrap to start of cyclic queue
        }
      }
      next_client_idx = (next_client_idx + 1) % num_clients;  // round robin polling for each client

      next_push = futureMillis(SYNC_PUSH_INTERVAL);
    }

    // TODO: periodically check for OLD/inactive entries in known_clients[], and evict
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

static char command[MAX_POST_TEXT_LEN+1];

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

  Serial.print("Room ID: ");
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
    char reply[160];
    the_mesh.handleAdminCommand(0, command, reply);  // NOTE: there is no sender_timestamp via serial!
    if (reply[0]) {
      Serial.print("  -> "); Serial.println(reply);
    }

    command[0] = 0;  // reset command buffer
  }

  the_mesh.loop();
}
