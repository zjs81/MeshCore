#pragma once

#include "CustomLR1110.h"
#include "RadioLibWrappers.h"

class CustomLR1110Wrapper : public RadioLibWrapper {
private:
  // Duty cycling state
  bool dutyCycleActive = false;
  uint32_t rxPeriodMs = 0;
  uint32_t sleepPeriodMs = 0;
  uint32_t dutyCycleStartMs = 0;
  bool inRxPhase = false;
  uint32_t lastActivityMs = 0;  // Track recent activity to prevent sleep during operation
  bool dutyCycleInitialized = false;

public:
  CustomLR1110Wrapper(CustomLR1110& radio, mesh::MainBoard& board) : RadioLibWrapper(radio, board) { }
  
  // Override startRecv to track activity and prevent aggressive duty cycling
  void startRecv() override {
    lastActivityMs = millis();
    // If duty cycling is active during a manual startRecv call, temporarily disable it
    if (dutyCycleActive && inRxPhase) {
      Serial.println("DEBUG: LR1110 pausing duty cycle for manual RX operation");
      ((CustomLR1110 *)_radio)->standby();
      delay(2);
    }
    RadioLibWrapper::startRecv();
  }
  
  bool isReceivingPacket() override { 
    if (((CustomLR1110 *)_radio)->isReceiving()) {
      lastActivityMs = millis();  // Track activity
      return true;
    }
    return false;
  }
  
  float getCurrentRSSI() override {
    float rssi = -110;
    ((CustomLR1110 *)_radio)->getRssiInst(&rssi);
    return rssi;
  }

  void onSendFinished() override {
    lastActivityMs = millis();  // Track activity
    RadioLibWrapper::onSendFinished();
    _radio->setPreambleLength(16); // overcomes weird issues with small and big pkts
  }

  float getLastRSSI() const override { return ((CustomLR1110 *)_radio)->getRSSI(); }
  float getLastSNR() const override { return ((CustomLR1110 *)_radio)->getSNR(); }
  int16_t setRxBoostedGainMode(bool en) { return ((CustomLR1110 *)_radio)->setRxBoostedGainMode(en); };

  // Hardware duty cycling support for LR1110 - custom implementation using timer-based sleep/wake
  bool supportsHardwareDutyCycle() const override { 
    return true; // LR1110 supports custom duty cycling implementation
  }

  bool startReceiveDutyCycle(uint32_t rxPeriod, uint32_t sleepPeriod) override {
    // Store duty cycle parameters (already in milliseconds)
    rxPeriodMs = rxPeriod;
    sleepPeriodMs = sleepPeriod;
    dutyCycleStartMs = millis();
    inRxPhase = true;
    dutyCycleActive = true;
    dutyCycleInitialized = true;
    lastActivityMs = millis();
    
    // Start the first RX period
    int16_t state = ((CustomLR1110 *)_radio)->startReceive();
    
    if (state == RADIOLIB_ERR_NONE) {
      Serial.printf("DEBUG: LR1110 custom duty cycle started: RX=%dms, Sleep=%dms\n", 
                    rxPeriod, sleepPeriod);
      return true;
    } else {
      Serial.printf("ERROR: LR1110 startReceive failed: %d\n", state);
      dutyCycleActive = false;
      return false;
    }
  }

  // Override the loop method to handle duty cycling
  void loop() override {
    // Call parent loop first
    RadioLibWrapper::loop();
    
    // Only manage duty cycling if it's been properly initialized and we're not in a critical period
    if (dutyCycleActive && dutyCycleInitialized) {
      // Don't manage duty cycling if there's been recent activity (prevents interference)
      uint32_t now = millis();
      if (now - lastActivityMs < 5000) {  // 5 second grace period after activity
        return;
      }
      
      manageDutyCycle();
    }
  }

  void manageDutyCycle() {
    uint32_t now = millis();
    uint32_t elapsed = now - dutyCycleStartMs;
    
    if (inRxPhase) {
      // We're in RX phase - check if it's time to sleep
      if (elapsed >= rxPeriodMs) {
        // Switch to sleep phase
        Serial.println("DEBUG: LR1110 entering sleep phase");
        ((CustomLR1110 *)_radio)->sleep();
        inRxPhase = false;
        dutyCycleStartMs = now;
      }
    } else {
      // We're in sleep phase - check if it's time to wake up
      if (elapsed >= sleepPeriodMs) {
        // Switch to RX phase
        Serial.println("DEBUG: LR1110 waking up for RX phase");
        ((CustomLR1110 *)_radio)->standby();
        delay(2); // Brief delay for radio to wake up
        ((CustomLR1110 *)_radio)->startReceive();
        inRxPhase = true;
        dutyCycleStartMs = now;
      }
    }
  }

  // Stop duty cycling and return to continuous RX mode
  void stopDutyCycle() {
    if (dutyCycleActive) {
      dutyCycleActive = false;
      ((CustomLR1110 *)_radio)->standby();
      delay(2);
      ((CustomLR1110 *)_radio)->startReceive();
      Serial.println("DEBUG: LR1110 duty cycling stopped, returning to continuous RX");
    }
  }

  // Override idle to stop duty cycling
  void idle() override {
    if (dutyCycleActive) {
      stopDutyCycle();
    }
    RadioLibWrapper::idle();
  }

  bool startChannelActivityDetection() override {
    // Use RadioLib's CAD functionality if available
    int16_t state = ((CustomLR1110 *)_radio)->scanChannel();
    
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("DEBUG: LR1110 CAD started");
      return true;
    } else {
      Serial.printf("ERROR: LR1110 CAD start failed: %d\n", state);
      return false;
    }
  }

  bool isChannelActivityDetected() override {
    // LR1110 specific implementation would go here
    // For now, return false as a placeholder
    return false;
  }

  bool startPreambleDetection() override {
    // Start receive mode with preamble detection
    int16_t state = ((CustomLR1110 *)_radio)->startReceive();
    
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("DEBUG: LR1110 preamble detection started");
      return true;
    } else {
      Serial.printf("ERROR: LR1110 preamble detection start failed: %d\n", state);
      return false;
    }
  }

  bool isPreambleDetected() override {
    // LR1110 specific implementation would go here
    // For now, return false as a placeholder
    return false;
  }
};
