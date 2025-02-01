#include "Dispatcher.h"

#if MESH_PACKET_LOGGING
  #include <Arduino.h>
#endif

namespace mesh {

void Dispatcher::begin() {
  n_sent_flood = n_sent_direct = 0;
  n_recv_flood = n_recv_direct = 0;
  n_full_events = 0;

  _radio->begin();
}

float Dispatcher::getAirtimeBudgetFactor() const {
  return 2.0;   // default, 33.3%  (1/3rd)
}

void Dispatcher::loop() {
  if (outbound) {  // waiting for outbound send to be completed
    if (_radio->isSendComplete()) {
      long t = _ms->getMillis() - outbound_start;
      total_air_time += t;  // keep track of how much air time we are using
      //Serial.print("  airtime="); Serial.println(t);

      // will need radio silence up to next_tx_time
      next_tx_time = futureMillis(t * getAirtimeBudgetFactor());

      _radio->onSendFinished();
      onPacketSent(outbound);
      if (outbound->isRouteFlood()) {
        n_sent_flood++;
      } else {
        n_sent_direct++;
      }
      outbound = NULL;
    } else if (millisHasNowPassed(outbound_expiry)) {
      MESH_DEBUG_PRINTLN("Dispatcher::loop(): WARNING: outbound packed send timed out!");

      _radio->onSendFinished();
      releasePacket(outbound);  // return to pool
      outbound = NULL;
    } else {
      return;  // can't do any more radio activity until send is complete or timed out
    }
  }

  checkRecv();
  checkSend();
}

void Dispatcher::onPacketSent(Packet* packet) {
  releasePacket(packet);  // default behaviour, return packet to pool
}

void Dispatcher::checkRecv() {
  Packet* pkt;
  {
    uint8_t raw[MAX_TRANS_UNIT];
    int len = _radio->recvRaw(raw, MAX_TRANS_UNIT);
    if (len > 0) {
      pkt = _mgr->allocNew();
      if (pkt == NULL) {
        MESH_DEBUG_PRINTLN("Dispatcher::checkRecv(): WARNING: received data, no unused packets available!");
      } else {
        int i = 0;
#ifdef NODE_ID
        uint8_t sender_id = raw[i++];
        if (sender_id == NODE_ID - 1 || sender_id == NODE_ID + 1) {  // simulate that NODE_ID can only hear NODE_ID-1 or NODE_ID+1, eg. 3 can't hear 1
        } else {
          _mgr->free(pkt);  // put back into pool
          return;
        }
#endif

        pkt->header = raw[i++];
        pkt->path_len = raw[i++];

        if (pkt->path_len > MAX_PATH_SIZE || i + pkt->path_len > len) {
          MESH_DEBUG_PRINTLN("Dispatcher::checkRecv(): partial or corrupt packet received, len=%d", len);
          _mgr->free(pkt);  // put back into pool
          pkt = NULL;
        } else {
          memcpy(pkt->path, &raw[i], pkt->path_len); i += pkt->path_len;

          pkt->payload_len = len - i;  // payload is remainder
          memcpy(pkt->payload, &raw[i], pkt->payload_len);
        }
      }
    } else {
      pkt = NULL;
    }
  }
  if (pkt) {
    if (pkt->isRouteFlood()) {
      n_recv_flood++;
    } else {
      n_recv_direct++;
    }
    #if MESH_PACKET_LOGGING
      Serial.printf("PACKET: recv, len=%d (type=%d, route=%s, payload_len=%d) SNR=%d RSSI=%d\n", 
            2 + pkt->path_len + pkt->payload_len, pkt->getPayloadType(), pkt->isRouteDirect() ? "D" : "F", pkt->payload_len,
            (int)_radio->getLastSNR(), (int)_radio->getLastRSSI());
    #endif
    DispatcherAction action = onRecvPacket(pkt);
    if (action == ACTION_RELEASE) {
      _mgr->free(pkt);
    } else if (action == ACTION_MANUAL_HOLD) {
      // sub-class is wanting to manually hold Packet instance, and call releasePacket() at appropriate time
    } else {   // ACTION_RETRANSMIT*
      uint8_t priority = (action >> 24) - 1;
      uint32_t _delay = action & 0xFFFFFF;

      _mgr->queueOutbound(pkt, priority, futureMillis(_delay));
    }
  }
}

void Dispatcher::checkSend() {
  if (_mgr->getOutboundCount() == 0) return;  // nothing waiting to send
  if (!millisHasNowPassed(next_tx_time)) return;   // still in 'radio silence' phase (from airtime budget setting)
  if (_radio->isReceiving()) return;  // LBT - check if radio is currently mid-receive, or if channel activity

  outbound = _mgr->getNextOutbound(_ms->getMillis());
  if (outbound) {
    int len = 0;
    uint8_t raw[MAX_TRANS_UNIT];

#ifdef NODE_ID
    raw[len++] = NODE_ID;
#endif
    raw[len++] = outbound->header;
    raw[len++] = outbound->path_len;
    memcpy(&raw[len], outbound->path, outbound->path_len); len += outbound->path_len;

    if (len + outbound->payload_len > MAX_TRANS_UNIT) {
      MESH_DEBUG_PRINTLN("Dispatcher::checkSend(): FATAL: Invalid packet queued... too long, len=%d", len + outbound->payload_len);
      _mgr->free(outbound);
      outbound = NULL;
    } else {
      memcpy(&raw[len], outbound->payload, outbound->payload_len); len += outbound->payload_len;

      uint32_t max_airtime = _radio->getEstAirtimeFor(len)*3/2;
      outbound_start = _ms->getMillis();
      _radio->startSendRaw(raw, len);
      outbound_expiry = futureMillis(max_airtime);

    #if MESH_PACKET_LOGGING
      Serial.printf("PACKET: send, len=%d (type=%d, route=%s, payload_len=%d)\n", 
            len, outbound->getPayloadType(), outbound->isRouteDirect() ? "D" : "F", outbound->payload_len);
    #endif
    }
  }
}

Packet* Dispatcher::obtainNewPacket() {
  auto pkt = _mgr->allocNew();  // TODO: zero out all fields
  if (pkt == NULL) {
    n_full_events++;
  } else {
    pkt->payload_len = pkt->path_len = 0;
  }
  return pkt;
}

void Dispatcher::releasePacket(Packet* packet) {
  _mgr->free(packet);
}

void Dispatcher::sendPacket(Packet* packet, uint8_t priority, uint32_t delay_millis) {
  if (packet->path_len > MAX_PATH_SIZE || packet->payload_len > MAX_PACKET_PAYLOAD) {
    MESH_DEBUG_PRINTLN("Dispatcher::sendPacket(): ERROR: invalid packet... path_len=%d, payload_len=%d", (uint32_t) packet->path_len, (uint32_t) packet->payload_len);
    _mgr->free(packet);
  } else {
    _mgr->queueOutbound(packet, priority, futureMillis(delay_millis));
  }
}

// Utility function -- handles the case where millis() wraps around back to zero
//   2's complement arithmetic will handle any unsigned subtraction up to HALF the word size (32-bits in this case)
bool Dispatcher::millisHasNowPassed(unsigned long timestamp) const {
  return (long)(_ms->getMillis() - timestamp) > 0;
}

unsigned long Dispatcher::futureMillis(int millis_from_now) const {
  return _ms->getMillis() + millis_from_now;
}

}