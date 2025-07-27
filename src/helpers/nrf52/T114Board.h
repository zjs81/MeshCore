#pragma once

#include <MeshCore.h>
#include <Arduino.h>
#include <helpers/NRFAdcCalibration.h>

// LoRa radio module pins for Heltec T114
#define  P_LORA_DIO_1     20
#define  P_LORA_NSS       24
#define  P_LORA_RESET     25
#define  P_LORA_BUSY      17
#define  P_LORA_SCLK      19
#define  P_LORA_MISO      23
#define  P_LORA_MOSI      22
#define  SX126X_POWER_EN  37

#define SX126X_DIO2_AS_RF_SWITCH  true
#define SX126X_DIO3_TCXO_VOLTAGE   1.8

// built-ins
#define  PIN_VBAT_READ    4
#define  PIN_BAT_CTL      6
#define  MV_LSB   (3000.0F / 4096.0F) // 12-bit ADC with 3.0V input range

class T114Board : public mesh::MainBoard {
protected:
  uint8_t startup_reason;

public:
  void begin();
  void loop();
  uint8_t getStartupReason() const override { return startup_reason; }

#if defined(P_LORA_TX_LED)
  void onBeforeTransmit() override {
    digitalWrite(P_LORA_TX_LED, LOW);   // turn TX LED on
  }
  void onAfterTransmit() override {
    digitalWrite(P_LORA_TX_LED, HIGH);   // turn TX LED off
  }
#endif

  uint16_t getBattMilliVolts() override {
    analogReadResolution(12);
    analogReference(AR_INTERNAL_3_0);
    pinMode(PIN_BAT_CTL, OUTPUT);          // battery adc can be read only ctrl pin 6 set to high
    digitalWrite(PIN_BAT_CTL, 1);

    delay(10);
    
    uint32_t raw = 0;
    for (int i = 0; i < 8; i++) {
      raw += analogRead(PIN_VBAT_READ);
    }
    raw = raw / 8;
    
    digitalWrite(PIN_BAT_CTL, 0);

    // Use calibrated ADC conversion with board multiplier (4.9 from original MV_LSB calculation)
    return NRFAdcCalibration::rawToMilliVolts(raw, 4.9f, 3.0f);
  }

  const char* getManufacturerName() const override {
    return "Heltec T114";
  }

  void reboot() override {
    NVIC_SystemReset();
  }

  bool startOTAUpdate(const char* id, char reply[]) override;
};
