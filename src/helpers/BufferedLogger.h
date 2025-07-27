#pragma once

#ifdef ARDUINO
  #include <Arduino.h>
#else
  #include <cstdint>
  #include <cstring>
#endif

#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  #include <InternalFileSystem.h>
  using File = Adafruit_LittleFS_Namespace::File;
  using FileSystem = Adafruit_LittleFS;
#elif defined(RP2040_PLATFORM)
  #include <LittleFS.h>
  typedef fs::File File;
  using FileSystem = fs::FS;
#elif defined(ESP32)
  // ESP32 can use either SPIFFS or LittleFS - auto-detect based on Arduino ESP32 core version
  #if defined(ARDUINO_ARCH_ESP32) && defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(2, 0, 0)
    // ESP32 Arduino Core 2.0+ prefers LittleFS
    #include <LittleFS.h>
    #define ESP32_USE_LITTLEFS
  #else
    // Older ESP32 Arduino Core or custom build - use SPIFFS
    #include <SPIFFS.h>
    #define ESP32_USE_SPIFFS
  #endif
  typedef fs::File File;
  using FileSystem = fs::FS;
#endif

#ifndef BUFFER_SIZE
  #define BUFFER_SIZE 512
#endif

#ifndef FLUSH_INTERVAL_MS
  #define FLUSH_INTERVAL_MS 5000  // Flush every 5 seconds
#endif

class BufferedLogger {
private:
  FileSystem* _fs;
  const char* _filename;
  char _buffer[BUFFER_SIZE];
  size_t _buffer_pos;
  unsigned long _last_flush;
  bool _enabled;

  File openAppend() {
  #if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
    return _fs->open(_filename, FILE_O_WRITE);
  #elif defined(RP2040_PLATFORM)
    return _fs->open(_filename, "a");
  #elif defined(ESP32)
    // ESP32 file opening - handle both SPIFFS and LittleFS
    #ifdef ESP32_USE_LITTLEFS
      return _fs->open(_filename, "a", true);  // LittleFS format
    #else
      // SPIFFS format - use string mode for better compatibility
      return _fs->open(_filename, "a");  
    #endif
  #else
    return _fs->open(_filename, "a", true);
  #endif
  }

  void forceFlush() {
    if (_buffer_pos > 0 && _enabled && isFileSystemReady()) {
      File f = openAppend();
      if (f) {
        f.write((const uint8_t*)_buffer, _buffer_pos);
        f.close();
      }
      _buffer_pos = 0;
      _last_flush = millis();
    }
  }

  bool isFileSystemReady() {
    if (!_fs) return false;
    
    #if defined(ESP32)
      // ESP32 - check if file system is mounted
      static bool fs_initialized = false;
      if (!fs_initialized) {
        #ifdef ESP32_USE_LITTLEFS
          fs_initialized = LittleFS.begin(true);  // Auto-format if needed
          if (fs_initialized) {
            _fs = &LittleFS;  // Update pointer to correct FS instance
          }
        #else
          fs_initialized = SPIFFS.begin(true);   // Auto-format if needed
          if (fs_initialized) {
            _fs = &SPIFFS;   // Update pointer to correct FS instance
          }
        #endif
      }
      return fs_initialized;
    #else
      // Other platforms - assume ready if fs pointer exists
      return true;
    #endif
  }

public:
  BufferedLogger(FileSystem* fs, const char* filename) 
    : _fs(fs), _filename(filename), _buffer_pos(0), _last_flush(0), _enabled(false) {
    #if defined(ESP32)
      // For ESP32, we'll initialize the FS pointer during first use
      // This allows the logger to work even if fs pointer is null initially
    #endif
  }

  void setEnabled(bool enabled) {
    if (!enabled && _enabled) {
      forceFlush();  // Flush any remaining data when disabling
    }
    _enabled = enabled;
  }

  void log(const char* message) {
    if (!_enabled) return;

    size_t msg_len = strlen(message);
    
    // Check if message fits in buffer
    if (_buffer_pos + msg_len >= BUFFER_SIZE) {
      forceFlush();  // Flush current buffer
    }
    
    // If message is still too large, write directly to file
    if (msg_len >= BUFFER_SIZE) {
      if (isFileSystemReady()) {
        File f = openAppend();
        if (f) {
          f.write(message, msg_len);
          f.close();
        }
      }
      return;
    }
    
    // Add message to buffer
    memcpy(&_buffer[_buffer_pos], message, msg_len);
    _buffer_pos += msg_len;
    
    // Check if we should flush due to time
    unsigned long current_time = millis();
    if (current_time - _last_flush >= FLUSH_INTERVAL_MS) {
      forceFlush();
    }
  }

  void printf(const char* format, ...) {
    if (!_enabled) return;

    va_list args;
    va_start(args, format);
    
    char temp_buffer[256];
    int len = vsnprintf(temp_buffer, sizeof(temp_buffer), format, args);
    va_end(args);
    
    if (len > 0 && len < sizeof(temp_buffer)) {
      log(temp_buffer);
    }
  }

  void loop() {
    // Call this periodically to ensure timely flushing
    if (_enabled && _buffer_pos > 0) {
      unsigned long current_time = millis();
      if (current_time - _last_flush >= FLUSH_INTERVAL_MS) {
        forceFlush();
      }
    }
  }

  void flush() {
    forceFlush();
  }

  void clear() {
    if (isFileSystemReady() && _fs->exists(_filename)) {
      _fs->remove(_filename);
    }
    _buffer_pos = 0;
  }

  // Specialized methods for packet logging
  void logRx(mesh::Packet* pkt, int len, float score) {
    if (!_enabled) return;
    
    char temp[256];
    const char* datetime = getLogDateTime();
    snprintf(temp, sizeof(temp), 
      "%s: RX, len=%d (type=%d, route=%s, payload_len=%d) SNR=%d score=%d",
      datetime, len, pkt->getPayloadType(), pkt->isRouteDirect() ? "D" : "F", 
      pkt->payload_len, (int)pkt->getSNR(), (int)(score*1000));
    
    log(temp);
    
    if (pkt->getPayloadType() == PAYLOAD_TYPE_PATH || pkt->getPayloadType() == PAYLOAD_TYPE_REQ
      || pkt->getPayloadType() == PAYLOAD_TYPE_RESPONSE || pkt->getPayloadType() == PAYLOAD_TYPE_TXT_MSG) {
      snprintf(temp, sizeof(temp), " [%02X -> %02X]\n", 
        (uint32_t)pkt->payload[1], (uint32_t)pkt->payload[0]);
    } else {
      strcpy(temp, "\n");
    }
    log(temp);
  }

  void logTx(mesh::Packet* pkt, int len) {
    if (!_enabled) return;
    
    char temp[256];
    const char* datetime = getLogDateTime();
    snprintf(temp, sizeof(temp),
      "%s: TX, len=%d (type=%d, route=%s, payload_len=%d)",
      datetime, len, pkt->getPayloadType(), pkt->isRouteDirect() ? "D" : "F", pkt->payload_len);
    
    log(temp);
    
    if (pkt->getPayloadType() == PAYLOAD_TYPE_PATH || pkt->getPayloadType() == PAYLOAD_TYPE_REQ
      || pkt->getPayloadType() == PAYLOAD_TYPE_RESPONSE || pkt->getPayloadType() == PAYLOAD_TYPE_TXT_MSG) {
      snprintf(temp, sizeof(temp), " [%02X -> %02X]\n", 
        (uint32_t)pkt->payload[1], (uint32_t)pkt->payload[0]);
    } else {
      strcpy(temp, "\n");
    }
    log(temp);
  }

  void logTxFail(mesh::Packet* pkt, int len) {
    if (!_enabled) return;
    
    char temp[256];
    const char* datetime = getLogDateTime();
    snprintf(temp, sizeof(temp),
      "%s: TX FAIL!, len=%d (type=%d, route=%s, payload_len=%d)\n",
      datetime, len, pkt->getPayloadType(), pkt->isRouteDirect() ? "D" : "F", pkt->payload_len);
    
    log(temp);
  }

private:
  const char* getLogDateTime() {
    static char tmp[32];
    // This would need to be implemented based on the actual RTC clock available
    // For now, return a simple timestamp
    unsigned long now = millis();
    snprintf(tmp, sizeof(tmp), "%lu", now);
    return tmp;
  }
}; 