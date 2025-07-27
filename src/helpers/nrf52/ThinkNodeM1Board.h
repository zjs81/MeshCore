#pragma once

#include <MeshCore.h>
#include <Arduino.h>
#include <helpers/NRFAdcCalibration.h>

// LoRa radio module pins for Elecrow ThinkNode M1
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
#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096

#define VBAT_DIVIDER      (0.5F)          // 150K + 150K voltage divider on VBAT
#define VBAT_DIVIDER_COMP (2.0F)          // Compensation factor for the VBAT divider

#define PIN_VBAT_READ     (4)
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

class ThinkNodeM1Board : public mesh::MainBoard {
protected:
  uint8_t startup_reason;

public:

  void begin();
  void loop();
  uint16_t getBattMilliVolts() override;
  bool startOTAUpdate(const char* id, char reply[]) override;

  uint8_t getStartupReason() const override {
    return startup_reason;
  }

  #if defined(P_LORA_TX_LED)
  void onBeforeTransmit() override {
    digitalWrite(P_LORA_TX_LED, HIGH);   // turn TX LED on
  }
  void onAfterTransmit() override {
    digitalWrite(P_LORA_TX_LED, LOW);   // turn TX LED off
  }
  #endif

  const char* getManufacturerName() const override {
    return "Elecrow ThinkNode-M1";
  }

  void reboot() override {
    NVIC_SystemReset();
  }
};
