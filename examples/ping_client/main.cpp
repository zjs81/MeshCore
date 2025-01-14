#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>
#include <SPIFFS.h>

#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/RadioLibWrappers.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>

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

#ifdef HELTEC_LORA_V3
  #include <helpers/HeltecV3Board.h>
  static HeltecV3Board board;
#else
  #error "need to provide a 'board' object"
#endif

/* ------------------------------ Code -------------------------------- */

class MyMesh : public mesh::Mesh {
  uint32_t last_advert_timestamp = 0;
  mesh::Identity server_id;
  uint8_t server_secret[PUB_KEY_SIZE];
  int server_path_len = -1;
  uint8_t server_path[MAX_PATH_SIZE];
  bool got_adv = false;

protected:
  int searchPeersByHash(const uint8_t* hash) override {
    if (got_adv && server_id.isHashMatch(hash)) {
      return 1;
    }
    return 0;  // not found
  }

  void onAdvertRecv(mesh::Packet* packet, const mesh::Identity& id, uint32_t timestamp, const uint8_t* app_data, size_t app_data_len) override {
    if (memcmp(app_data, "PING", 4) == 0) {
      Serial.println("Received advertisement from a PING server");

      // check for replay attacks
      if (timestamp > last_advert_timestamp) {
        last_advert_timestamp = timestamp;

        server_id = id;
        self_id.calcSharedSecret(server_secret, id);  // calc ECDH shared secret
        got_adv = true;
      }
    }
  }

  void onPeerDataRecv(mesh::Packet* packet, uint8_t type, int sender_idx, uint8_t* data, size_t len) override {
    if (type == PAYLOAD_TYPE_RESPONSE) {
      Serial.println("Received PING Reply!");

      if (packet->isRouteFlood()) {
        // let server know path TO here, so they can use sendDirect() for future ping responses
        mesh::Packet* path = createPathReturn(server_id, server_secret, packet->path, packet->path_len, 0, NULL, 0);
        if (path) sendFlood(path);
      }
    }
  }

  void onPeerPathRecv(mesh::Packet* packet, int sender_idx, uint8_t* path, uint8_t path_len, uint8_t extra_type, uint8_t* extra, uint8_t extra_len) override {
    // must be from server_id 
    Serial.printf("PATH to server, path_len=%d\n", (uint32_t) path_len);

    memcpy(server_path, path, server_path_len = path_len);  // store a copy of path, for sendDirect()

    if (packet->isRouteFlood()) {
      // send a reciprocal return path to sender, but send DIRECTLY!
      mesh::Packet* rpath = createPathReturn(server_id, server_secret, packet->path, packet->path_len, 0, NULL, 0);
      if (rpath) sendDirect(rpath, path, path_len);
    }

    if (extra_type == PAYLOAD_TYPE_RESPONSE) {
      Serial.println("Received PING Reply!");
    }
  }

public:
  MyMesh(mesh::Radio& radio, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : mesh::Mesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables)
  {
  }

  void sendPingPacket() {
    if (!got_adv) return;   // have not received Advert yet

    uint32_t now = getRTCClock()->getCurrentTime(); // important, need timestamp in packet, so that packet_hash will be unique
    mesh::Packet* ping = createAnonDatagram(PAYLOAD_TYPE_ANON_REQ, self_id, server_id, server_secret, (uint8_t *) &now, sizeof(now));

    if (ping) {
      if (server_path_len < 0) {
        sendFlood(ping);
      } else {
        sendDirect(ping, server_path, server_path_len);
      }
    }
  }

};

SPIClass spi;
StdRNG fast_rng;
SimpleMeshTables tables;
SX1262 radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, spi);
MyMesh the_mesh(*new RadioLibWrapper(radio, board), fast_rng, *new VolatileRTCClock(), tables);
unsigned long nextPing;

void halt() {
  while (1) ;
}

void setup() {
  Serial.begin(115200);

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
  the_mesh.begin();

  RadioNoiseListener true_rng(radio);
  the_mesh.self_id = mesh::LocalIdentity(&true_rng);  // create new random identity

  nextPing = 0;
}

void loop() {
  if (the_mesh.millisHasNowPassed(nextPing)) {
    the_mesh.sendPingPacket();

    nextPing = the_mesh.futureMillis(10000);  // attempt ping every 10 seconds
  }
  the_mesh.loop();
}
