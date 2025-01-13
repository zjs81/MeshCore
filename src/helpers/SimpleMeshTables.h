#pragma once

#include <MeshTables.h>

#ifdef ESP32
  #include <FS.h>
#endif

#define MAX_PACKET_HASHES  64

class SimpleMeshTables : public mesh::MeshTables {
  uint8_t _fwd_hashes[MAX_PACKET_HASHES*MAX_HASH_SIZE];
  int _next_fwd_idx;

  int lookupHashIndex(const uint8_t* hash) const {
    const uint8_t* sp = _fwd_hashes;
    for (int i = 0; i < MAX_PACKET_HASHES; i++, sp += MAX_HASH_SIZE) {
      if (memcmp(hash, sp, MAX_HASH_SIZE) == 0) return i;
    }
    return -1;
  }

public:
  SimpleMeshTables() { 
    memset(_fwd_hashes, 0, sizeof(_fwd_hashes));
    _next_fwd_idx = 0;
  }

#ifdef ESP32
  void restoreFrom(File f) {
    f.read(_fwd_hashes, sizeof(_fwd_hashes));
    f.read((uint8_t *) &_next_fwd_idx, sizeof(_next_fwd_idx));
  }
  void saveTo(File f) {
    f.write(_fwd_hashes, sizeof(_fwd_hashes));
    f.write((const uint8_t *) &_next_fwd_idx, sizeof(_next_fwd_idx));
  }
#endif

  bool hasForwarded(const uint8_t* packet_hash) const override {
    int i = lookupHashIndex(packet_hash);
    return i >= 0;
  }

  void setHasForwarded(const uint8_t* packet_hash) override {
    int i = lookupHashIndex(packet_hash);
    if (i >= 0) {
      // already in table
    } else {
      memcpy(&_fwd_hashes[_next_fwd_idx*MAX_HASH_SIZE], packet_hash, MAX_HASH_SIZE);

      _next_fwd_idx = (_next_fwd_idx + 1) % MAX_PACKET_HASHES;  // cyclic table
    }
  }

};
