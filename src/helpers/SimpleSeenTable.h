#pragma once

#include <Packet.h>
#include <string.h>

#define MAX_PACKET_HASHES  64

class SimpleSeenTable {
  uint8_t _hashes[MAX_PACKET_HASHES*MAX_HASH_SIZE];
  int _next_idx;

public:
  SimpleSeenTable() { 
    memset(_hashes, 0, sizeof(_hashes));
    _next_idx = 0;
  }

  bool hasSeenPacket(const mesh::Packet* packet) {
    uint8_t hash[MAX_HASH_SIZE];
    packet->calculatePacketHash(hash);

    const uint8_t* sp = _hashes;
    for (int i = 0; i < MAX_PACKET_HASHES; i++, sp += MAX_HASH_SIZE) {
      if (memcmp(hash, sp, MAX_HASH_SIZE) == 0) return true;
    }

    memcpy(&_hashes[_next_idx*MAX_HASH_SIZE], hash, MAX_HASH_SIZE);
    _next_idx = (_next_idx + 1) % MAX_PACKET_HASHES;  // cyclic table

    return false;
  }

};
