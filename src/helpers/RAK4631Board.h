#pragma once

#include <MeshCore.h>
#include <Arduino.h>

#if defined(NRF52_PLATFORM)

// LoRa radio module pins for RAK4631
#define  P_LORA_DIO_1   47
#define  P_LORA_NSS     42
#define  P_LORA_RESET  RADIOLIB_NC   // 38
#define  P_LORA_BUSY    46
#define  P_LORA_SCLK    43
#define  P_LORA_MISO    45
#define  P_LORA_MOSI    44
#define  SX126X_POWER_EN  37
 
#define SX126X_DIO2_AS_RF_SWITCH  true
#define SX126X_DIO3_TCXO_VOLTAGE   1.8

// built-ins
#define  PIN_VBAT_READ    1
#define  ADC_MULTIPLIER   6.17

class RAK4631Board : public mesh::MainBoard {
protected:
  uint8_t startup_reason;

public:
  void begin() {
    // for future use, sub-classes SHOULD call this from their begin()
    startup_reason = BD_STARTUP_NORMAL;

    pinMode(SX126X_POWER_EN, OUTPUT);
    digitalWrite(SX126X_POWER_EN, HIGH);
  }

  uint8_t getStartupReason() const override { return startup_reason; }

  #define BATTERY_SAMPLES 10

  uint16_t getBattMilliVolts() override {
    analogReadResolution(12);

    uint32_t raw = 0;
    for (int i = 0; i < BATTERY_SAMPLES; i++) {
      raw += analogRead(PIN_VBAT_READ);
    }
    raw = raw / BATTERY_SAMPLES;

    return (ADC_MULTIPLIER * raw) / 4096;
  }

  const char* getManufacturerName() const override {
    return "RAK 4631";
  }

  void reboot() override {
    NVIC_SystemReset();
  }
};

#endif
