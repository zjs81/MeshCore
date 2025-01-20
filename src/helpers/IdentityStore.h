#pragma once

#if defined(ESP32)
  #include <FS.h>
  #define FILESYSTEM  fs::FS
#elif defined(NRF52_PLATFORM)
  #include <Adafruit_LittleFS.h>
  #define FILESYSTEM  Adafruit_LittleFS

  using namespace Adafruit_LittleFS_Namespace;
#endif

#include <Identity.h>

class IdentityStore {
  FILESYSTEM* _fs;
  const char* _dir;
public:
  IdentityStore(FILESYSTEM& fs, const char* dir): _fs(&fs), _dir(dir) { }

  void begin() { _fs->mkdir(_dir); }
  bool load(const char *name, mesh::LocalIdentity& id);
  bool save(const char *name, const mesh::LocalIdentity& id);
};
