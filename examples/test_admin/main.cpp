#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>
#include <SPIFFS.h>

#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/CustomSX1262Wrapper.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/StaticPoolPacketManager.h>

/* ---------------------------------- CONFIGURATION ------------------------------------- */

#ifndef LORA_FREQ
  #define LORA_FREQ   915.0
#endif
#ifndef LORA_BW
  #define LORA_BW     125
#endif
#ifndef LORA_SF
  #define LORA_SF     9
#endif
#ifndef LORA_CR
  #define LORA_CR      5
#endif

#define  ADMIN_PASSWORD  "h^(kl@#)"

#ifdef HELTEC_LORA_V3
  #include <helpers/HeltecV3Board.h>
  static HeltecV3Board board;
#else
  #error "need to provide a 'board' object"
#endif

/* -------------------------------------------------------------------------------------- */

#define MAX_TEXT_LEN    (10*CIPHER_BLOCK_SIZE)  // must be LESS than (MAX_PACKET_PAYLOAD - FROM_HASH_LEN - CIPHER_MAC_SIZE - 1)

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

class MyMesh : public mesh::Mesh {
  uint32_t last_advert_timestamp = 0;
  mesh::Identity server_id;
  uint8_t server_secret[PUB_KEY_SIZE];
  int server_path_len = -1;
  uint8_t server_path[MAX_PATH_SIZE];
  bool got_adv = false;

protected:
  void onAdvertRecv(mesh::Packet* packet, const mesh::Identity& id, uint32_t timestamp, const uint8_t* app_data, size_t app_data_len) override {
    if (memcmp(app_data, "repeater:", 9) == 0) {
      Serial.println("Received advertisement from a repeater!");

      // check for replay attacks
      if (timestamp > last_advert_timestamp) {
        last_advert_timestamp = timestamp;

        server_id = id;
        self_id.calcSharedSecret(server_secret, id);  // calc ECDH shared secret
        got_adv = true;

        // 'login' to repeater. (mainly lets it know our public key)
        uint32_t now = getRTCClock()->getCurrentTime(); // important, need timestamp in packet, so that packet_hash will be unique
        uint8_t temp[4 + 8];
        memcpy(temp, &now, 4);
        memcpy(&temp[4], ADMIN_PASSWORD, 8);

        mesh::Packet* login = createAnonDatagram(PAYLOAD_TYPE_ANON_REQ, self_id, server_id, server_secret, temp, sizeof(temp));
        if (login) sendFlood(login);  // server_path won't be known yet
      }
    }
  }

  void handleResponse(const uint8_t* reply, size_t reply_len) {
    if (reply_len >= 4 + sizeof(RepeaterStats)) {      // got an GET_STATS reply from repeater
      RepeaterStats stats;
      memcpy(&stats, &reply[4], sizeof(stats));
      Serial.println("Repeater Stats:");
      Serial.printf("  battery: %d mV\n", (uint32_t) stats.batt_milli_volts);
      Serial.printf("  tx queue: %d\n", (uint32_t) stats.curr_tx_queue_len);
      Serial.printf("  free queue: %d\n", (uint32_t) stats.curr_free_queue_len);
      Serial.printf("  last RSSI: %d\n", (int) stats.last_rssi);
      Serial.printf("  num recv: %d\n", stats.n_packets_recv);
      Serial.printf("  num sent: %d\n", stats.n_packets_sent);
      Serial.printf("  air time (secs): %d\n", stats.total_air_time_secs);
      Serial.printf("  up time (secs): %d\n", stats.total_up_time_secs);
    } else if (reply_len > 4) {   // got an SET_* reply from repeater
      char tmp[MAX_PACKET_PAYLOAD];
      memcpy(tmp, &reply[4], reply_len - 4);
      tmp[reply_len - 4] = 0;  // make a C string of reply

      Serial.print("Reply: "); Serial.println(tmp);
    }
  }

  int searchPeersByHash(const uint8_t* hash) override {
    if (got_adv && server_id.isHashMatch(hash)) {
      return 1;
    }
    return 0;  // not found
  }

  void getPeerSharedSecret(uint8_t* dest_secret, int peer_idx) override {
    // lookup pre-calculated shared_secret
    memcpy(dest_secret, server_secret, PUB_KEY_SIZE);
  }

  void onPeerDataRecv(mesh::Packet* packet, uint8_t type, int sender_idx, const uint8_t* secret, uint8_t* data, size_t len) override {
    if (type == PAYLOAD_TYPE_RESPONSE) {
      handleResponse(data, len);

      if (packet->isRouteFlood()) {
        // let server know path TO here, so they can use sendDirect() for future ping responses
        mesh::Packet* path = createPathReturn(server_id, secret, packet->path, packet->path_len, 0, NULL, 0);
        if (path) sendFlood(path);
      }
    }
  }

  bool onPeerPathRecv(mesh::Packet* packet, int sender_idx, const uint8_t* secret, uint8_t* path, uint8_t path_len, uint8_t extra_type, uint8_t* extra, uint8_t extra_len) override {
    // must be from server_id 
    Serial.printf("PATH to repeater, path_len=%d\n", (uint32_t) path_len);

    memcpy(server_path, path, server_path_len = path_len);  // store a copy of path, for sendDirect()

    if (extra_type == PAYLOAD_TYPE_RESPONSE) {
      handleResponse(extra, extra_len);
    }
    return true;  // send reciprocal path if necessary
  }

public:
  MyMesh(mesh::Radio& radio, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : mesh::Mesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables)
  {
  }

  mesh::Packet* createStatsRequest(uint32_t max_age) {
    uint8_t payload[9];
    uint32_t now = getRTCClock()->getCurrentTime();
    memcpy(payload, &now, 4);
    payload[4] = CMD_GET_STATS;
    memcpy(&payload[5], &max_age, 4);

    return createDatagram(PAYLOAD_TYPE_REQ, server_id, server_secret, payload, sizeof(payload));
  }

  mesh::Packet* createSetClockRequest(uint32_t timestamp) {
    uint8_t payload[9];
    uint32_t now = getRTCClock()->getCurrentTime();
    memcpy(payload, &now, 4);
    payload[4] = CMD_SET_CLOCK;
    memcpy(&payload[5], &now, 4);  // repeated :-(

    return createDatagram(PAYLOAD_TYPE_REQ, server_id, server_secret, payload, sizeof(payload));
  }

  mesh::Packet* createSetAirtimeFactorRequest(float airtime_factor) {
    uint8_t payload[16];
    uint32_t now = getRTCClock()->getCurrentTime();
    memcpy(payload, &now, 4);
    payload[4] = CMD_SET_CONFIG;
    sprintf((char *) &payload[5], "AF%f", airtime_factor);

    return createDatagram(PAYLOAD_TYPE_REQ, server_id, server_secret, payload, sizeof(payload));
  }

  mesh::Packet* createAnnounceRequest() {
    uint8_t payload[5];
    uint32_t now = getRTCClock()->getCurrentTime();
    memcpy(payload, &now, 4);
    payload[4] = CMD_SEND_ANNOUNCE;

    return createDatagram(PAYLOAD_TYPE_REQ, server_id, server_secret, payload, sizeof(payload));
  }

  mesh::Packet* parseCommand(char* command) {
    if (strcmp(command, "stats") == 0) {
      return createStatsRequest(60*60);    // max_age = one hour
    } else if (memcmp(command, "setclock ", 9) == 0) {
      uint32_t timestamp = atol(&command[9]);
      return createSetClockRequest(timestamp);
    } else if (memcmp(command, "set AF=", 7) == 0) {
      float factor = atof(&command[7]);
      return createSetAirtimeFactorRequest(factor);
    } else if (strcmp(command, "ann") == 0) {
      return createAnnounceRequest();
    }
    return NULL;  // unknown command
  }

  void sendCommand(mesh::Packet* pkt) {
    if (server_path_len < 0) {
      sendFlood(pkt);
    } else {
      sendDirect(pkt, server_path, server_path_len);
    }
  }
};

StdRNG fast_rng;
SimpleMeshTables tables;
#if defined(P_LORA_SCLK)
SPIClass spi;
CustomSX1262 radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, spi);
#else
CustomSX1262 radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY);
#endif
MyMesh the_mesh(*new CustomSX1262Wrapper(radio, board), fast_rng, *new VolatileRTCClock(), tables);

void halt() {
  while (1) ;
}

static char command[MAX_TEXT_LEN+1];

#include <SHA256.h>

void setup() {
  Serial.begin(115200);
  delay(5000);

  board.begin();
#if defined(P_LORA_SCLK)
  spi.begin(P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI);
  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22, 8);
#else
  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22, 8);
#endif
  if (status != RADIOLIB_ERR_NONE) {
    Serial.print("ERROR: radio init failed: ");
    Serial.println(status);
    halt();
  }

  fast_rng.begin(radio.random(0x7FFFFFFF));

/* add this to tests
  uint8_t mac_encrypted[CIPHER_MAC_SIZE+CIPHER_BLOCK_SIZE];
  const char *orig_msg = "original";
  int enc_len = mesh::Utils::encryptThenMAC(mesh.admin_secret, mac_encrypted, (const uint8_t *) orig_msg, strlen(orig_msg));
  char decrypted[CIPHER_BLOCK_SIZE*2];
  int len = mesh::Utils::MACThenDecrypt(mesh.admin_secret, (uint8_t *)decrypted, mac_encrypted, enc_len);
  if (len > 0) {
    decrypted[len] = 0;
    Serial.print("decrypted text: "); Serial.println(decrypted);
  } else {
    Serial.println("MACs DONT match!");
  }
*/

  Serial.println("Help:");
  Serial.println("  enter 'key' to generate new keypair");
  Serial.println("  enter 'stats' to request repeater stats");
  Serial.println("  enter 'setclock {unix-epoch-seconds}' to set repeater's clock");
  Serial.println("  enter 'set AF={factor}' to set airtime budget factor");
  Serial.println("  enter 'ann' to make repeater re-announce to mesh");

  the_mesh.begin();

  command[0] = 0;
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

    if (strcmp(command, "key") == 0) {
      mesh::LocalIdentity new_id(the_mesh.getRNG());
      new_id.printTo(Serial);
    } else {
      mesh::Packet* pkt = the_mesh.parseCommand(command);
      if (pkt) { 
        the_mesh.sendCommand(pkt);
        Serial.println("   (request sent)");
      } else {
        Serial.print("   ERROR: unknown command: "); Serial.println(command);
      }
    }
    command[0] = 0;  // reset command buffer
  }

  the_mesh.loop();
}
