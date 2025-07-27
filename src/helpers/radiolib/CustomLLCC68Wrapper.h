#pragma once

#include "RadioLibWrappers.h"
#include "CustomLLCC68.h"

// SX126x IRQ flags from datasheet
#define SX126X_IRQ_CAD_DETECTED     0x0100  // CAD detected
#define SX126X_IRQ_CAD_DONE         0x0080  // CAD done

class CustomLLCC68Wrapper : public RadioLibWrapper {
private:
  CustomLLCC68 *sx;

public:
  CustomLLCC68Wrapper(CustomLLCC68 &radio, mesh::MainBoard &board) : RadioLibWrapper(radio, board) {
    sx = &radio;
  }

  bool isReceivingPacket() override {
    return sx->isReceiving();
  }

  float getCurrentRSSI() override {
    return sx->getRSSI();
  }

  // Hardware duty cycling support for LLCC68 - now with real RadioLib API
  bool supportsHardwareDutyCycle() const override { 
    return true; 
  }

  bool startReceiveDutyCycle(uint32_t rxPeriod, uint32_t sleepPeriod) override {
    // Convert from milliseconds to microseconds for RadioLib API
    uint32_t rxPeriodUs = rxPeriod * 1000;
    uint32_t sleepPeriodUs = sleepPeriod * 1000;
    
    // Use RadioLib's hardware duty cycling
    int16_t state = sx->startReceiveDutyCycle(rxPeriodUs, sleepPeriodUs);
    
    if (state == RADIOLIB_ERR_NONE) {
      Serial.printf("DEBUG: LLCC68 hardware duty cycle started: RX=%dms, Sleep=%dms\n", 
                    rxPeriod, sleepPeriod);
      return true;
    } else {
      Serial.printf("ERROR: LLCC68 startReceiveDutyCycle failed: %d\n", state);
      return false;
    }
  }

  bool startChannelActivityDetection() override {
    // Use RadioLib's CAD functionality
    int16_t state = sx->scanChannel();
    
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("DEBUG: LLCC68 CAD started");
      return true;
    } else {
      Serial.printf("ERROR: LLCC68 CAD start failed: %d\n", state);
      return false;
    }
  }

  bool isChannelActivityDetected() override {
    // Check if there's activity detected using correct method name
    uint16_t irqFlags = sx->getIrqFlags();
    bool cadDetected = (irqFlags & SX126X_IRQ_CAD_DETECTED) != 0;
    
    if (cadDetected) {
      Serial.println("DEBUG: LLCC68 CAD activity detected");
    }
    
    return cadDetected;
  }

  bool startPreambleDetection() override {
    // Start receive mode with preamble detection
    int16_t state = sx->startReceive();
    
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("DEBUG: LLCC68 preamble detection started");
      return true;
    } else {
      Serial.printf("ERROR: LLCC68 preamble detection start failed: %d\n", state);
      return false;
    }
  }

  bool isPreambleDetected() override {
    // Check for preamble detection IRQ using correct method name
    uint16_t irqFlags = sx->getIrqFlags();
    bool preambleDetected = (irqFlags & SX126X_IRQ_PREAMBLE_DETECTED) != 0;
    
    if (preambleDetected) {
      Serial.println("DEBUG: LLCC68 preamble detected");
    }
    
    return preambleDetected;
  }
};
