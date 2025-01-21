#pragma once

#include "CustomSX1268.h"
#include "RadioLibWrappers.h"

class CustomSX1268Wrapper : public RadioLibWrapper {
public:
  CustomSX1268Wrapper(CustomSX1268& radio, mesh::MainBoard& board) : RadioLibWrapper(radio, board) { }
  bool isReceiving() override { 
    if (((CustomSX1268 *)_radio)->isReceiving()) return true;

#if 0  //  has BUG :-(  commenting out for now
    idle();  // put sx126x into standby
    // do some basic CAD (blocks for ~12780 micros (on SF 10)!)
    if (((CustomSX1268 *)_radio)->scanChannel() == RADIOLIB_LORA_DETECTED) return true;
#endif
    return false;
  }
  float getLastRSSI() const override { return ((CustomSX1268 *)_radio)->getRSSI(); }
  float getLastSNR() const override { return ((CustomSX1268 *)_radio)->getSNR(); }
};
