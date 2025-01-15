#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>
#include <SPIFFS.h>

#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/RadioLibWrappers.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>

/* ---------------------------------- CONFIGURATION ------------------------------------- */

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

//#define RUN_AS_ALICE    true

#if RUN_AS_ALICE
  const char* alice_private = "B8830658388B2DDF22C3A508F4386975970CDE1E2A2A495C8F3B5727957A97629255A1392F8BA4C26A023A0DAB78BFC64D261C8E51507496DD39AFE3707E7B42";
#else
  const char *bob_private = "30BAA23CCB825D8020A59C936D0AB7773B07356020360FC77192813640BAD375E43BBF9A9A7537E4B9614610F1F2EF874AAB390BA9B0C2F01006B01FDDFEFF0C";
#endif
  const char *alice_public = "106A5136EC0DD797650AD204C065CF9B66095F6ED772B0822187785D65E11B1F";
  const char *bob_public = "020BCEDAC07D709BD8507EC316EB5A7FF2F0939AF5057353DCE7E4436A1B9681";

#ifdef HELTEC_LORA_V3
  #include <helpers/HeltecV3Board.h>
  static HeltecV3Board board;
#else
  #error "need to provide a 'board' object"
#endif

#define SEND_TIMEOUT_BASE_MILLIS          300
#define FLOOD_SEND_TIMEOUT_FACTOR         16.0f
#define DIRECT_SEND_PERHOP_FACTOR         3.7213f
#define DIRECT_SEND_PERHOP_EXTRA_MILLIS   100

/* -------------------------------------------------------------------------------------- */

static unsigned long txt_send_timeout;

#define MAX_CONTACTS         1
#define MAX_SEARCH_RESULTS   1

#define MAX_TEXT_LEN    (10*CIPHER_BLOCK_SIZE)  // must be LESS than (MAX_PACKET_PAYLOAD - 4 - CIPHER_MAC_SIZE - 1)

struct ContactInfo {
  mesh::Identity id;
  const char* name;
  int out_path_len;
  uint8_t out_path[MAX_PATH_SIZE];
  uint32_t last_advert_timestamp;
  uint8_t shared_secret[PUB_KEY_SIZE];
};

class MyMesh : public mesh::Mesh {
public:
  ContactInfo contacts[MAX_CONTACTS];
  int num_contacts;

  void addContact(const char* name, const mesh::Identity& id) {
    if (num_contacts < MAX_CONTACTS) {
      contacts[num_contacts].id = id;
      contacts[num_contacts].name = name;
      contacts[num_contacts].last_advert_timestamp = 0;
      contacts[num_contacts].out_path_len = -1;
      // only need to calculate the shared_secret once, for better performance
      self_id.calcSharedSecret(contacts[num_contacts].shared_secret, id);
      num_contacts++;
    }
  }

protected:
  int  matching_peer_indexes[MAX_SEARCH_RESULTS];

  int searchPeersByHash(const uint8_t* hash) override {
    int n = 0;
    for (int i = 0; i < num_contacts && n < MAX_SEARCH_RESULTS; i++) {
      if (contacts[i].id.isHashMatch(hash)) {
        matching_peer_indexes[n++] = i;  // store the INDEXES of matching contacts (for subsequent 'peer' methods)
      }
    }
    return n;
  }

  void onAdvertRecv(mesh::Packet* packet, const mesh::Identity& id, uint32_t timestamp, const uint8_t* app_data, size_t app_data_len) override {
    Serial.print("Valid Advertisement -> ");
    mesh::Utils::printHex(Serial, id.pub_key, PUB_KEY_SIZE);
    Serial.println();

    for (int i = 0; i < num_contacts; i++) {
      ContactInfo& from = contacts[i];
      // check for replay attacks 
      if (id.matches(from.id) && timestamp > from.last_advert_timestamp) {  // is from one of our contacts
        from.last_advert_timestamp = timestamp;
        Serial.printf("   From contact: %s\n", from.name);
      }
    }
  }

  void getPeerSharedSecret(uint8_t* dest_secret, int peer_idx) override {
    int i = matching_peer_indexes[peer_idx];
    if (i >= 0 && i < num_contacts) {
      // lookup pre-calculated shared_secret
      memcpy(dest_secret, contacts[i].shared_secret, PUB_KEY_SIZE);
    } else {
      MESH_DEBUG_PRINTLN("getPeerSHharedSecret: Invalid peer idx: %d", i);
    }
  }

  void onPeerDataRecv(mesh::Packet* packet, uint8_t type, int sender_idx, uint8_t* data, size_t len) override {
    if (type == PAYLOAD_TYPE_TXT_MSG && len > 5) {
      int i = matching_peer_indexes[sender_idx];
      if (i < 0 || i >= num_contacts) {
        MESH_DEBUG_PRINTLN("onPeerDataRecv: Invalid sender idx: %d", i);
        return;
      }

      ContactInfo& from = contacts[i];

      uint32_t timestamp;
      memcpy(&timestamp, data, 4);  // timestamp (by sender's RTC clock - which could be wrong)
      uint flags = data[4];   // message attempt number, and other flags

      // len can be > original length, but 'text' will be padded with zeroes
      data[len] = 0; // need to make a C string again, with null terminator

      //if ( ! alreadyReceived timestamp ) {
      Serial.printf("(%s) MSG -> from %s\n", packet->isRouteFlood() ? "FLOOD" : "DIRECT", from.name);
      Serial.printf("   %s\n", (const char *) &data[5]);
      //}

      uint32_t ack_hash;    // calc truncated hash of the message timestamp + text + sender pub_key, to prove to sender that we got it
      mesh::Utils::sha256((uint8_t *) &ack_hash, 4, data, 5 + strlen((char *)&data[5]), from.id.pub_key, PUB_KEY_SIZE);

      if (packet->isRouteFlood()) {
        // let this sender know path TO here, so they can use sendDirect(), and ALSO encode the ACK
        mesh::Packet* path = createPathReturn(from.id, from.shared_secret, packet->path, packet->path_len,
                                              PAYLOAD_TYPE_ACK, (uint8_t *) &ack_hash, 4);
        if (path) sendFlood(path);
      } else {
        mesh::Packet* ack = createAck(ack_hash);
        if (ack) {
          if (from.out_path_len < 0) {
            sendFlood(ack);
          } else {
            sendDirect(ack, from.out_path, from.out_path_len);
          }
        }
      }
    }
  }

  void onPeerPathRecv(mesh::Packet* packet, int sender_idx, uint8_t* path, uint8_t path_len, uint8_t extra_type, uint8_t* extra, uint8_t extra_len) override {
    int i = matching_peer_indexes[sender_idx];
    if (i < 0 || i >= num_contacts) {
      MESH_DEBUG_PRINTLN("onPeerPathRecv: Invalid sender idx: %d", i);
      return;
    }

    ContactInfo& from = contacts[i];
    Serial.printf("PATH to: %s, path_len=%d\n", from.name, (uint32_t) path_len);

    // NOTE: for this impl, we just replace the current 'out_path' regardless, whenever sender sends us a new out_path.
    // FUTURE: could store multiple out_paths per contact, and try to find which is the 'best'(?)
    memcpy(from.out_path, path, from.out_path_len = path_len);  // store a copy of path, for sendDirect()

    if (packet->isRouteFlood()) {
      // send a reciprocal return path to sender, but send DIRECTLY!
      mesh::Packet* rpath = createPathReturn(from.id, from.shared_secret, packet->path, packet->path_len, 0, NULL, 0);
      if (rpath) sendDirect(rpath, path, path_len);
    }

    if (extra_type == PAYLOAD_TYPE_ACK && extra_len >= 4) {
      // also got an encoded ACK!
      processAck(extra);
    }
  }

  void onAckRecv(mesh::Packet* packet, uint32_t ack_crc) override {
    processAck((uint8_t *)&ack_crc);
  }

  void processAck(const uint8_t *data) {
    if (memcmp(data, &expected_ack_crc, 4) == 0) {     // got an ACK from recipient
      Serial.printf("   Got ACK! (round trip: %d millis)\n", _ms->getMillis() - last_msg_sent);
      // NOTE: the same ACK can be received multiple times!
      expected_ack_crc = 0;  // reset our expected hash, now that we have received ACK
      txt_send_timeout = 0;
    } else {
      uint32_t crc;
      memcpy(&crc, data, 4);
      MESH_DEBUG_PRINTLN(" unknown ACK received: %08X (expected: %08X)", crc, expected_ack_crc);
    }
  }

public:
  uint32_t expected_ack_crc;
  unsigned long last_msg_sent;

  MyMesh(mesh::Radio& radio, mesh::RNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables)
     : mesh::Mesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables)
  {
    num_contacts = 0;
  }

  mesh::Packet* composeMsgPacket(ContactInfo& recipient, uint8_t attempt, const char *text) {
    int text_len = strlen(text);
    if (text_len > MAX_TEXT_LEN) return NULL;

    uint8_t temp[5+MAX_TEXT_LEN+1];
    uint32_t timestamp = getRTCClock()->getCurrentTime();
    memcpy(temp, &timestamp, 4);   // mostly an extra blob to help make packet_hash unique
    temp[4] = attempt;
    memcpy(&temp[5], text, text_len + 1);

    // calc expected ACK reply
    mesh::Utils::sha256((uint8_t *)&expected_ack_crc, 4, temp, 5 + text_len, self_id.pub_key, PUB_KEY_SIZE);
    last_msg_sent = _ms->getMillis();

    return createDatagram(PAYLOAD_TYPE_TXT_MSG, recipient.id, recipient.shared_secret, temp, 5 + text_len);
  }

  void sendSelfAdvert() {
    mesh::Packet* adv = createAdvert(self_id);
    if (adv) {
      sendFlood(adv);
      Serial.println("   (advert sent).");
    } else {
      Serial.println("   ERROR: unable to create packet.");
    }
  }
};

SPIClass spi;
StdRNG fast_rng;
SimpleMeshTables tables;
SX1262 radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, spi);
MyMesh the_mesh(*new RadioLibWrapper(radio, board), fast_rng, *new VolatileRTCClock(), tables);

void halt() {
  while (1) ;
}

static char command[MAX_TEXT_LEN+1];

void setup() {
  Serial.begin(115200);

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

  fast_rng.begin(radio.random(0x7FFFFFFF));

#if RUN_AS_ALICE
  Serial.println("   --- user: Alice ---");
  the_mesh.self_id = mesh::LocalIdentity(alice_private, alice_public);
  the_mesh.addContact("Bob", mesh::Identity(bob_public));
#else
  Serial.println("   --- user: Bob ---");
  the_mesh.self_id = mesh::LocalIdentity(bob_private, bob_public);
  the_mesh.addContact("Alice", mesh::Identity(alice_public));
#endif
  Serial.println("Help:");
  Serial.println("  enter 'adv' to advertise presence to mesh");
  Serial.println("  enter 'send {message text}' to send a message");

  the_mesh.begin();

  command[0] = 0;
  txt_send_timeout = 0;

  // send out initial Advertisement to the mesh
  the_mesh.sendSelfAdvert();
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

    if (memcmp(command, "send ", 5) == 0) {
      // TODO: some way to select recipient??
      ContactInfo& recipient = the_mesh.contacts[0];  // just send to first contact for now

      const char *text = &command[5];
      mesh::Packet* pkt = the_mesh.composeMsgPacket(recipient, 0, text);
      if (pkt) {
        uint32_t t = radio.getTimeOnAir(pkt->payload_len + pkt->path_len + 2) / 1000;

        if (recipient.out_path_len < 0) {
          the_mesh.sendFlood(pkt);
          txt_send_timeout = the_mesh.futureMillis(SEND_TIMEOUT_BASE_MILLIS + (FLOOD_SEND_TIMEOUT_FACTOR * t));
          Serial.printf("   (message sent - FLOOD, t=%d)\n", t);
        } else {
          the_mesh.sendDirect(pkt, recipient.out_path, recipient.out_path_len);

          txt_send_timeout = the_mesh.futureMillis(SEND_TIMEOUT_BASE_MILLIS + 
                     ( (t*DIRECT_SEND_PERHOP_FACTOR + DIRECT_SEND_PERHOP_EXTRA_MILLIS) * (recipient.out_path_len + 1)));

          Serial.printf("   (message sent - DIRECT, t=%d)\n", t);
        }
      } else {
        Serial.println("   ERROR: unable to create packet.");
      }
    } else if (strcmp(command, "adv") == 0) {
      the_mesh.sendSelfAdvert();
    } else if (strcmp(command, "key") == 0) {
      mesh::LocalIdentity new_id(the_mesh.getRNG());
      new_id.printTo(Serial);
    } else {
      Serial.print("   ERROR: unknown command: "); Serial.println(command);
    }

    command[0] = 0;  // reset command buffer
  }

  if (txt_send_timeout && the_mesh.millisHasNowPassed(txt_send_timeout)) {
    // failed to get an ACK
    ContactInfo& recipient = the_mesh.contacts[0];  // just the one contact for now
    Serial.println("   ERROR: timed out, no ACK.");

    // path to our contact is now possibly broken, fallback to Flood mode
    recipient.out_path_len = -1;

    txt_send_timeout = 0;
  }

  the_mesh.loop();
}
