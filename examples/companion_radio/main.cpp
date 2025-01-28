#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#if defined(NRF52_PLATFORM)
  #include <InternalFileSystem.h>
#elif defined(ESP32)
  #include <SPIFFS.h>
#endif

#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/RadioLibWrappers.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/IdentityStore.h>
#include <helpers/BaseSerialInterface.h>
#include <RTClib.h>

/* ---------------------------------- CONFIGURATION ------------------------------------- */

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

#ifndef MAX_CONTACTS
  #define MAX_CONTACTS         100
#endif

#include <helpers/BaseChatMesh.h>

#define SEND_TIMEOUT_BASE_MILLIS          300
#define FLOOD_SEND_TIMEOUT_FACTOR         16.0f
#define DIRECT_SEND_PERHOP_FACTOR         4.0f
#define DIRECT_SEND_PERHOP_EXTRA_MILLIS   200

#define  PUBLIC_GROUP_PSK  "izOH6cXN6mrJ5e26oRXNcg=="

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

// Believe it or not, this std C function is busted on some platforms!
static uint32_t _atoi(const char* sp) {
  uint32_t n = 0;
  while (*sp && *sp >= '0' && *sp <= '9') {
    n *= 10;
    n += (*sp++ - '0');
  }
  return n;
}

/*------------ Frame Protocol --------------*/

#define CMD_APP_START              1
#define CMD_SEND_TXT_MSG           2
#define CMD_SEND_CHANNEL_TXT_MSG   3
#define CMD_GET_CONTACTS           4   // with optional 'since' (for efficient sync)
#define CMD_GET_DEVICE_TIME        5
#define CMD_SET_DEVICE_TIME        6
#define CMD_SEND_SELF_ADVERT       7
#define CMD_SET_ADVERT_NAME        8
#define CMD_ADD_UPDATE_CONTACT     9
#define CMD_SYNC_NEXT_MESSAGE     10

#define RESP_CODE_OK                0
#define RESP_CODE_ERR               1
#define RESP_CODE_CONTACTS_START    2   // first reply to CMD_GET_CONTACTS
#define RESP_CODE_CONTACT           3   // multiple of these (after CMD_GET_CONTACTS)
#define RESP_CODE_END_OF_CONTACTS   4   // last reply to CMD_GET_CONTACTS
#define RESP_CODE_SELF_INFO         5   // reply to CMD_APP_START
#define RESP_CODE_SENT              6   // reply to CMD_SEND_TXT_MSG
#define RESP_CODE_CONTACT_MSG_RECV  7   // a reply to CMD_SYNC_NEXT_MESSAGE
#define RESP_CODE_CHANNEL_MSG_RECV  8   // a reply to CMD_SYNC_NEXT_MESSAGE

// these are _pushed_ to client app at any time
#define PUSH_CODE_ADVERT            0x80
#define PUSH_CODE_PATH_UPDATED      0x81
#define PUSH_CODE_SEND_CONFIRMED    0x82
#define PUSH_CODE_MSG_WAITING       0x83

/* -------------------------------------------------------------------------------------- */

class MyMesh : public BaseChatMesh {
  FILESYSTEM* _fs;
  uint32_t expected_ack_crc;  // TODO: keep table of expected ACKs
  mesh::GroupChannel* _public;
  BaseSerialInterface* _serial;
  unsigned long last_msg_sent;
  ContactsIterator _iter;
  uint32_t _iter_filter_since;
  bool  _iter_started;
  uint8_t cmd_frame[MAX_FRAME_SIZE+1];
  uint8_t out_frame[MAX_FRAME_SIZE+1];

  void loadContacts() {
    if (_fs->exists("/contacts2")) {
      File file = _fs->open("/contacts2");
      if (file) {
        bool full = false;
        while (!full) {
          ContactInfo c;
          uint8_t pub_key[32];
          uint8_t unused;
          uint32_t reserved;

          bool success = (file.read(pub_key, 32) == 32);
          success = success && (file.read((uint8_t *) &c.name, 32) == 32);
          success = success && (file.read(&c.type, 1) == 1);
          success = success && (file.read(&c.flags, 1) == 1);
          success = success && (file.read(&unused, 1) == 1);
          success = success && (file.read((uint8_t *) &reserved, 4) == 4);
          success = success && (file.read((uint8_t *) &c.out_path_len, 1) == 1);
          success = success && (file.read((uint8_t *) &c.last_advert_timestamp, 4) == 4);
          success = success && (file.read(c.out_path, 64) == 64);
          success = success && (file.read((uint8_t *) c.lastmod, 4) == 4);

          if (!success) break;  // EOF

          c.id = mesh::Identity(pub_key);
          if (!addContact(c)) full = true;
        }
        file.close();
      }
    }
  }

  void saveContacts() {
#if defined(NRF52_PLATFORM)
    File file = _fs->open("/contacts2", FILE_O_WRITE);
    if (file) { file.seek(0); file.truncate(); }
#else
    File file = _fs->open("/contacts2", "w", true);
#endif
    if (file) {
      ContactsIterator iter;
      ContactInfo c;
      uint8_t unused = 0;
      uint32_t reserved = 0;

      while (iter.hasNext(this, c)) {
        bool success = (file.write(c.id.pub_key, 32) == 32);
        success = success && (file.write((uint8_t *) &c.name, 32) == 32);
        success = success && (file.write(&c.type, 1) == 1);
        success = success && (file.write(&c.flags, 1) == 1);
        success = success && (file.write(&unused, 1) == 1);
        success = success && (file.write((uint8_t *) &reserved, 4) == 4);
        success = success && (file.write((uint8_t *) &c.out_path_len, 1) == 1);
        success = success && (file.write((uint8_t *) &c.last_advert_timestamp, 4) == 4);
        success = success && (file.write(c.out_path, 64) == 64);
        success = success && (file.write((uint8_t *) &c.lastmod, 4) == 4);

        if (!success) break;  // write failed
      }
      file.close();
    }
  }

  void writeOKFrame() {
    uint8_t buf[1];
    buf[0] = RESP_CODE_OK;
    _serial->writeFrame(buf, 1);
  }
  void writeErrFrame() {
    uint8_t buf[1];
    buf[0] = RESP_CODE_ERR;
    _serial->writeFrame(buf, 1);
  }

  void writeContactRespFrame(uint8_t code, const ContactInfo& contact) {
    int i = 0;
    out_frame[i++] = code;
    memcpy(&out_frame[i], contact.id.pub_key, PUB_KEY_SIZE); i += PUB_KEY_SIZE;
    out_frame[i++] = contact.type;
    out_frame[i++] = contact.flags;
    out_frame[i++] = contact.out_path_len;
    memcpy(&out_frame[i], contact.out_path, MAX_PATH_SIZE); i += MAX_PATH_SIZE;
    memcpy(&out_frame[i], contact.name, 32); i += 32;
    memcpy(&out_frame[i], &contact.last_advert_timestamp, 4); i += 4;
    memcpy(&out_frame[i], &contact.lastmod, 4); i += 4;
    _serial->writeFrame(out_frame, i);
  }

  void updateContactFromFrame(ContactInfo& contact, const uint8_t* frame) {
    int i = 0;
    uint8_t code = frame[i++];  // eg. CMD_ADD_UPDATE_CONTACT
    memcpy(contact.id.pub_key, &frame[i], PUB_KEY_SIZE); i += PUB_KEY_SIZE;
    contact.type = frame[i++];
    contact.flags = frame[i++];
    contact.out_path_len = frame[i++];
    memcpy(contact.out_path, &frame[i], MAX_PATH_SIZE); i += MAX_PATH_SIZE;
    memcpy(contact.name, &frame[i], 32); i += 32;
    memcpy(&contact.last_advert_timestamp, &frame[i], 4); i += 4;
  }

  void addToOfflineQueue(const uint8_t frame[], int len) {
    // TODO
  }
  int getFromOfflineQueue(uint8_t frame[]) {
    return 0;  // queue is empty
  }

  void soundBuzzer() {
    // TODO
  }

protected:
  void onDiscoveredContact(ContactInfo& contact, bool is_new) override {
    if (_serial->isConnected()) {
      out_frame[0] = PUSH_CODE_ADVERT;
      memcpy(&out_frame[1], contact.id.pub_key, PUB_KEY_SIZE);
      _serial->writeFrame(out_frame, 1 + PUB_KEY_SIZE);
    } else {
      soundBuzzer();
    }

    saveContacts();
  }

  void onContactPathUpdated(const ContactInfo& contact) override {
    out_frame[0] = PUSH_CODE_PATH_UPDATED;
    memcpy(&out_frame[1], contact.id.pub_key, PUB_KEY_SIZE);
    _serial->writeFrame(out_frame, 1 + PUB_KEY_SIZE);   // NOTE: app may not be connected

    saveContacts();
  }

  bool processAck(const uint8_t *data) override {
    // TODO: see if matches any in a table
    if (memcmp(data, &expected_ack_crc, 4) == 0) {     // got an ACK from recipient
      out_frame[0] = PUSH_CODE_SEND_CONFIRMED;
      memcpy(&out_frame[1], data, 4);
      uint32_t trip_time = _ms->getMillis() - last_msg_sent;
      memcpy(&out_frame[5], &trip_time, 4);
      _serial->writeFrame(out_frame, 9);

      // NOTE: the same ACK can be received multiple times!
      expected_ack_crc = 0;  // reset our expected hash, now that we have received ACK
      return true;
    }
    return false;
  }

  void onMessageRecv(const ContactInfo& from, uint8_t path_len, uint32_t sender_timestamp, const char *text) override {
    int i = 0;
    out_frame[i++] = RESP_CODE_CONTACT_MSG_RECV;
    memcpy(&out_frame[i], from.id.pub_key, 6); i += 6;  // just 6-byte prefix
    out_frame[i++] = path_len;
    out_frame[i++] = TXT_TYPE_PLAIN;
    memcpy(&out_frame[i], &sender_timestamp, 4); i += 4;
    int tlen = strlen(text);   // TODO: UTF-8 ??
    if (i + tlen > MAX_FRAME_SIZE) {
      tlen = MAX_FRAME_SIZE - i;
    }
    memcpy(&out_frame[i], text, tlen); i += tlen;
    addToOfflineQueue(out_frame, i);

    if (_serial->isConnected()) {
      uint8_t frame[1];
      frame[0] = PUSH_CODE_MSG_WAITING;  // send push 'tickle'
      _serial->writeFrame(frame, 1);
    } else {
      soundBuzzer();
    }
  }

  void onChannelMessageRecv(const mesh::GroupChannel& channel, int in_path_len, uint32_t timestamp, const char *text) override {
    int i = 0;
    out_frame[i++] = RESP_CODE_CHANNEL_MSG_RECV;
    out_frame[i++] = in_path_len < 0 ? 0xFF : in_path_len;
    out_frame[i++] = TXT_TYPE_PLAIN;
    memcpy(&out_frame[i], &timestamp, 4); i += 4;
    int tlen = strlen(text);   // TODO: UTF-8 ??
    if (i + tlen > MAX_FRAME_SIZE) {
      tlen = MAX_FRAME_SIZE - i;
    }
    memcpy(&out_frame[i], text, tlen); i += tlen;
    addToOfflineQueue(out_frame, i);

    if (_serial->isConnected()) {
      uint8_t frame[1];
      frame[0] = PUSH_CODE_MSG_WAITING;  // send push 'tickle'
      _serial->writeFrame(frame, 1);
    } else {
      soundBuzzer();
    }
  }

  uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override {
    return SEND_TIMEOUT_BASE_MILLIS + (FLOOD_SEND_TIMEOUT_FACTOR * pkt_airtime_millis);
  }
  uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override {
    return SEND_TIMEOUT_BASE_MILLIS + 
         ( (pkt_airtime_millis*DIRECT_SEND_PERHOP_FACTOR + DIRECT_SEND_PERHOP_EXTRA_MILLIS) * (path_len + 1));
  }

  void onSendTimeout() override {
    Serial.println("   ERROR: timed out, no ACK.");
  }

public:
  char self_name[sizeof(ContactInfo::name)];

  MyMesh(RadioLibWrapper& radio, mesh::RNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables)
     : BaseChatMesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables), _serial(NULL)
  {
    _iter_started = false;
  }

  void begin(FILESYSTEM& fs, BaseSerialInterface& serial) {
    _fs = &fs;
    _serial = &serial;

    BaseChatMesh::begin();

    strcpy(self_name, "UNSET");
    IdentityStore store(fs, "/identity");
    if (!store.load("_main", self_id, self_name, sizeof(self_name))) {
      self_id = mesh::LocalIdentity(getRNG());  // create new random identity
      store.save("_main", self_id);
    }

    loadContacts();
    _public = addChannel(PUBLIC_GROUP_PSK); // pre-configure Andy's public channel
  }

  void handleCmdFrame(size_t len) {
    if (cmd_frame[0] == CMD_APP_START && len >= 8) {   // sent when app establishes connection, respond with node ID
      uint8_t app_ver = cmd_frame[1];
      //  cmd_frame[2..7]  reserved future
      char* app_name = (char *) &cmd_frame[8];
      cmd_frame[len] = 0;  // make app_name null terminated
      MESH_DEBUG_PRINTLN("App %s connected, ver: %d", app_name, (uint32_t)app_ver);

      _iter_started = false;   // stop any left-over ContactsIterator
      int i = 0;
      out_frame[i++] = RESP_CODE_SELF_INFO;
      out_frame[i++] = ADV_TYPE_CHAT;   // what this node Advert identifies as (maybe node's pronouns too?? :-)
      out_frame[i++] = 0;  // reserved
      out_frame[i++] = 0;  // reserved
      memcpy(&out_frame[i], self_id.pub_key, PUB_KEY_SIZE); i += PUB_KEY_SIZE;
      int32_t latlonsats = 0;
      memcpy(&out_frame[i], &latlonsats, 4); i += 4;   // reserved future, for companion radios with GPS (like T-Beam, T1000)
      memcpy(&out_frame[i], &latlonsats, 4); i += 4;
      memcpy(&out_frame[i], &latlonsats, 4); i += 4;
      int tlen = strlen(self_name);   // revisit: UTF_8 ??
      memcpy(&out_frame[i], self_name, tlen); i += tlen;
      _serial->writeFrame(out_frame, i);
    } else if (cmd_frame[0] == CMD_SEND_TXT_MSG && len >= 9) {
      int i = 1;
      uint8_t attempt_and_flags = cmd_frame[i++];
      uint32_t msg_timestamp;
      memcpy(&msg_timestamp, &cmd_frame[i], 4); i += 4;
      uint8_t* pub_key_prefix = &cmd_frame[i]; i += 6;
      ContactInfo* recipient = lookupContactByPubKey(pub_key_prefix, 6);
      if (recipient && (attempt_and_flags >> 2) == TXT_TYPE_PLAIN) {
        char *text = (char *) &cmd_frame[i];
        int tlen = len - i;
        text[tlen] = 0;  // ensure null
        int result = sendMessage(*recipient, msg_timestamp, attempt_and_flags, text, expected_ack_crc);
        // TODO: add expected ACK to table
        if (result == MSG_SEND_FAILED) {
          writeErrFrame();
        } else {
          last_msg_sent = _ms->getMillis();

          out_frame[0] = RESP_CODE_SENT;
          out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
          memcpy(&out_frame[2], &expected_ack_crc, 4);
          _serial->writeFrame(out_frame, 6);
        }
      } else {
        writeErrFrame(); // unknown recipient, or unsuported TXT_TYPE_*
      }
    } else if (cmd_frame[0] == CMD_SEND_CHANNEL_TXT_MSG) {  // send GroupChannel msg
  #if 0  //TODO
      uint8_t temp[5+MAX_TEXT_LEN+32];
      uint32_t timestamp = getRTCClock()->getCurrentTime();
      memcpy(temp, &timestamp, 4);   // mostly an extra blob to help make packet_hash unique
      temp[4] = 0;  // attempt and flags

      sprintf((char *) &temp[5], "%s: %s", self_name, &command[7]);  // <sender>: <msg>
      temp[5 + MAX_TEXT_LEN] = 0;  // truncate if too long

      int len = strlen((char *) &temp[5]);
      auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, *_public, temp, 5 + len);
      if (pkt) {
        sendFlood(pkt);
        Serial.println("   Sent.");
      } else {
        Serial.println("   ERROR: unable to send");
      }
  #else
      writeErrFrame();
  #endif
    } else if (cmd_frame[0] == CMD_GET_CONTACTS) {  // get Contact list
      if (_iter_started) {
        writeErrFrame();   // iterator is currently busy
      } else {
        if (len >= 5) {   // has optional 'since' param
          memcpy(&_iter_filter_since, &cmd_frame[1], 4);
        } else {
          _iter_filter_since = 0;
        }

        uint8_t reply[5];
        reply[0] = RESP_CODE_CONTACTS_START;
        uint32_t count = getNumContacts();  // total, NOT filtered count
        memcpy(&reply[1], &count, 4);
        _serial->writeFrame(reply, 5);

        // start iterator
        _iter = startContactsIterator();
        _iter_started = true;
      }
    } else if (cmd_frame[0] == CMD_SET_ADVERT_NAME && len >= 2) {
      int nlen = len - 1;
      if (nlen > sizeof(self_name)-1) nlen = sizeof(self_name)-1;  // max len
      memcpy(self_name, &cmd_frame[1], nlen);
      self_name[nlen] = 0;
      IdentityStore store(*_fs, "/identity");       // update IdentityStore
      store.save("_main", self_id, self_name);
      writeOKFrame();
    } else if (cmd_frame[0] == CMD_GET_DEVICE_TIME) {
      uint8_t reply[5];
      reply[0] = RESP_CODE_OK;
      uint32_t now = getRTCClock()->getCurrentTime();
      memcpy(&reply[1], &now, 4);
      _serial->writeFrame(reply, 5);
    } else if (cmd_frame[0] == CMD_SET_DEVICE_TIME && len >= 5) {
      uint32_t secs;
      memcpy(&secs, &cmd_frame[1], 4);
      uint32_t curr = getRTCClock()->getCurrentTime();
      if (secs > curr) {
        getRTCClock()->setCurrentTime(secs);
        writeOKFrame();
      } else {
        writeErrFrame();
      }
    } else if (cmd_frame[0] == CMD_SEND_SELF_ADVERT) {
      auto pkt = createSelfAdvert(self_name);
      if (pkt) {
        if (len >= 2 && cmd_frame[1] == 1) {   // optional param (1 = flood, 0 = zero hop)
          sendFlood(pkt);
        } else {
          sendZeroHop(pkt);
        }
        writeOKFrame();
      } else {
        writeErrFrame();
      }
    } else if (cmd_frame[0] == CMD_ADD_UPDATE_CONTACT && len >= 1+32+2+1) {
      uint8_t* pub_key = &cmd_frame[1];
      ContactInfo* recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
      if (recipient) {
        updateContactFromFrame(*recipient, cmd_frame);
        //recipient->lastmod = ??   shouldn't be needed, app already has this version of contact
        saveContacts();
        writeOKFrame();
      } else {
        ContactInfo contact;
        updateContactFromFrame(contact, cmd_frame);
        contact.lastmod = getRTCClock()->getCurrentTime();
        if (addContact(contact)) {
          saveContacts();
          writeOKFrame();
        } else {
          writeErrFrame();  // table is full!
        }
      }
    } else if (cmd_frame[0] == CMD_SYNC_NEXT_MESSAGE) {
      int out_len;
      if ((out_len = getFromOfflineQueue(out_frame)) > 0) {
        _serial->writeFrame(out_frame, out_len);
      }
    } else {
      writeErrFrame();
      MESH_DEBUG_PRINTLN("ERROR: unknown command: %02X", cmd_frame[0]);
    }
  }

  void loop() {
    BaseChatMesh::loop();
    size_t len = _serial->checkRecvFrame(cmd_frame);
    if (len > 0) {
      handleCmdFrame(len);
    } else if (_iter_started    // check if our ContactsIterator is 'running'
      && !_serial->isWriteBusy()    // don't spam the Serial Interface too quickly!
    ) {
      ContactInfo contact;
      if (_iter.hasNext(this, contact)) {
        if (contact.lastmod > _iter_filter_since) {  // apply the 'since' filter
          writeContactRespFrame(RESP_CODE_CONTACT, contact);
        }
      } else {  // EOF
        out_frame[0] = RESP_CODE_END_OF_CONTACTS;
        _serial->writeFrame(out_frame, 1);
        _iter_started = false;
      }
    }
  }
};

#ifdef ESP32
  #ifdef BLE_PIN_CODE
    #include <helpers/esp32/SerialBLEInterface.h>
    SerialBLEInterface serial_interface;
  #else
    #include <helpers/ArduinoSerialInterface.h>
    ArduinoSerialInterface serial_interface;
  #endif
#elif defined(NRF52_PLATFORM)
  #ifdef BLE_PIN_CODE
    #error "BLE not defined yet"
  #else
    #include <helpers/ArduinoSerialInterface.h>
    ArduinoSerialInterface serial_interface;
  #endif
#else
  #error "need to define a serial interface"
#endif

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
MyMesh the_mesh(*new WRAPPER_CLASS(radio, board), fast_rng, *new VolatileRTCClock(), tables);

void halt() {
  while (1) ;
}

void setup() {
  Serial.begin(115200);

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

  fast_rng.begin(radio.random(0x7FFFFFFF));

#if defined(NRF52_PLATFORM)
  InternalFS.begin();

  the_mesh.begin(InternalFS, serial_interface);
#elif defined(ESP32)
  SPIFFS.begin(true);

#ifdef BLE_PIN_CODE
  serial_interface.begin("MeshCore", BLE_PIN_CODE);
#else
  serial_interface.begin(Serial);
#endif
  serial_interface.enable();

  the_mesh.begin(SPIFFS, serial_interface);
#else
  #error "need to define filesystem"
#endif
}

void loop() {
  the_mesh.loop();
}
