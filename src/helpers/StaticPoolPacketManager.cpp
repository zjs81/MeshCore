#include "StaticPoolPacketManager.h"

PacketQueue::PacketQueue(int max_entries) {
  _table = new mesh::Packet*[max_entries];
  _pri_table = new uint8_t[max_entries];
  _schedule_table = new uint32_t[max_entries];
  _size = max_entries;
  _num = 0;
  _head = 0;  // Add circular buffer support
}

int PacketQueue::countBefore(uint32_t now) const {
  int n = 0;
  for (int j = 0; j < _num; j++) {
    int idx = (_head + j) % _size;
    if (_schedule_table[idx] > now) continue;   // scheduled for future... ignore for now
    n++;
  }
  return n;
}

mesh::Packet* PacketQueue::get(uint32_t now) {
  uint8_t min_pri = 0xFF;
  int best_idx = -1;
  int best_pos = -1;
  
  for (int j = 0; j < _num; j++) {
    int idx = (_head + j) % _size;
    if (_schedule_table[idx] > now) continue;   // scheduled for future... ignore for now
    if (_pri_table[idx] < min_pri) {  // select most important priority amongst non-future entries
      min_pri = _pri_table[idx];
      best_idx = idx;
      best_pos = j;
    }
  }
  if (best_idx < 0) return NULL;   // empty, or all items are still in the future

  mesh::Packet* top = _table[best_idx];
  
  // Optimized removal - use circular buffer approach to minimize data movement
  if (best_pos == 0) {
    // Removing head element - just advance head pointer
    _head = (_head + 1) % _size;
  } else {
    // Shift only the elements before the removed item
    for (int i = best_pos; i > 0; i--) {
      int dst_idx = (_head + i) % _size;
      int src_idx = (_head + i - 1) % _size;
      _table[dst_idx] = _table[src_idx];
      _pri_table[dst_idx] = _pri_table[src_idx];
      _schedule_table[dst_idx] = _schedule_table[src_idx];
    }
    _head = (_head + 1) % _size;
  }
  _num--;
  return top;
}

mesh::Packet* PacketQueue::removeByIdx(int i) {
  if (i >= _num) return NULL;  // invalid index

  int actual_idx = (_head + i) % _size;
  mesh::Packet* item = _table[actual_idx];
  
  // Optimized removal similar to get()
  if (i == 0) {
    _head = (_head + 1) % _size;
  } else {
    for (int j = i; j > 0; j--) {
      int dst_idx = (_head + j) % _size;
      int src_idx = (_head + j - 1) % _size;
      _table[dst_idx] = _table[src_idx];
      _pri_table[dst_idx] = _pri_table[src_idx];
      _schedule_table[dst_idx] = _schedule_table[src_idx];
    }
    _head = (_head + 1) % _size;
  }
  _num--;
  return item;
}

void PacketQueue::add(mesh::Packet* packet, uint8_t priority, uint32_t scheduled_for) {
  if (_num == _size) {
    // TODO: log "FATAL: queue is full!"
    return;
  }
  
  int idx = (_head + _num) % _size;
  _table[idx] = packet;
  _pri_table[idx] = priority;
  _schedule_table[idx] = scheduled_for;
  _num++;
}

mesh::Packet* PacketQueue::itemAt(int i) {
  if (i >= _num) return NULL;
  int idx = (_head + i) % _size;
  return _table[idx];
}

StaticPoolPacketManager::StaticPoolPacketManager(int pool_size): unused(pool_size), send_queue(pool_size), rx_queue(pool_size) {
  // load up our unusued Packet pool
  for (int i = 0; i < pool_size; i++) {
    unused.add(new mesh::Packet(), 0, 0);
  }
}

mesh::Packet* StaticPoolPacketManager::allocNew() {
  return unused.removeByIdx(0);  // just get first one (returns NULL if empty)
}

void StaticPoolPacketManager::free(mesh::Packet* packet) {
  unused.add(packet, 0, 0);
}

void StaticPoolPacketManager::queueOutbound(mesh::Packet* packet, uint8_t priority, uint32_t scheduled_for) {
  send_queue.add(packet, priority, scheduled_for);
}

mesh::Packet* StaticPoolPacketManager::getNextOutbound(uint32_t now) {
  //send_queue.sort();   // sort by scheduled_for/priority first
  return send_queue.get(now);
}

int  StaticPoolPacketManager::getOutboundCount(uint32_t now) const {
  return send_queue.countBefore(now);
}

int StaticPoolPacketManager::getFreeCount() const {
  return unused.count();
}

mesh::Packet* StaticPoolPacketManager::getOutboundByIdx(int i) {
  return send_queue.itemAt(i);
}
mesh::Packet* StaticPoolPacketManager::removeOutboundByIdx(int i) {
  return send_queue.removeByIdx(i);
}

void StaticPoolPacketManager::queueInbound(mesh::Packet* packet, uint32_t scheduled_for) {
  rx_queue.add(packet, 0, scheduled_for);
}
mesh::Packet* StaticPoolPacketManager::getNextInbound(uint32_t now) {
  return rx_queue.get(now);
}

int StaticPoolPacketManager::getInboundCount(uint32_t now) const {
  return rx_queue.countBefore(now);
}
