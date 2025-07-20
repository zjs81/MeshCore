#pragma once

#include <MeshCore.h>
#include <Arduino.h>

class SenseCapSolarBoard : public mesh::MainBoard {
protected:
  uint8_t startup_reason;

public:
  void begin();
  void loop();
  uint8_t getStartupReason() const override { return startup_reason; }

#if defined(P_LORA_TX_LED)
  void onBeforeTransmit() override {
    digitalWrite(P_LORA_TX_LED, HIGH);   // turn TX LED on
  }
  void onAfterTransmit() override {
    digitalWrite(P_LORA_TX_LED, LOW);   // turn TX LED off
  }
#endif

  uint16_t getBattMilliVolts() override {
    digitalWrite(VBAT_ENABLE, LOW);
    int adcvalue = 0;
    analogReadResolution(12);
    analogReference(AR_INTERNAL_3_0);
    delay(10);
    adcvalue = analogRead(BATTERY_PIN);
    return (adcvalue * ADC_MULTIPLIER * AREF_VOLTAGE) / 4.096;
  }

  const char* getManufacturerName() const override {
    return "Seeed SenseCap Solar";
  }

  void reboot() override {
    NVIC_SystemReset();
  }

  bool startOTAUpdate(const char* id, char reply[]) override;
};
