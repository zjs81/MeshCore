#pragma once

#include "CustomLLCC68.h"
#include "RadioLibWrappers.h"

class CustomLLCC68Wrapper : public RadioLibWrapper {
public:
  CustomLLCC68Wrapper(CustomLLCC68& radio, mesh::MainBoard& board) : RadioLibWrapper(radio, board) { }
  bool isReceiving() override { 
    if (((CustomLLCC68 *)_radio)->isReceiving()) return true;

    idle();  // put sx126x into standby
    // do some basic CAD (blocks for ~12780 micros (on SF 10)!)
    bool activity = (((CustomLLCC68 *)_radio)->scanChannel() == RADIOLIB_LORA_DETECTED);
    idle();

    return activity;
  }
  float getLastRSSI() const override { return ((CustomLLCC68 *)_radio)->getRSSI(); }
  float getLastSNR() const override { return ((CustomLLCC68 *)_radio)->getSNR(); }

  float packetScore(float snr, int packet_len) override {
    int sf = ((CustomLLCC68 *)_radio)->spreadingFactor;
    const float A = 0.7;
    const float B = 0.4;
  
    float ber = exp(-pow(10, (snr / 10)) / (A * pow(10, (snr / 10)) + B * (1 << sf)));

    return pow(1 - ber, packet_len * 8);
  }
};
