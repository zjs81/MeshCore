#pragma once

#include <MeshCore.h>
#include <Arduino.h>

// LoRa radio module pins
#define P_LORA_DIO_1 (32 + 10)
#define P_LORA_NSS (32 + 13)
#define P_LORA_RESET (32 + 15)
#define P_LORA_BUSY (32 + 11)
#define P_LORA_SCLK (0 + 12)
#define P_LORA_MISO (32 + 9)
#define P_LORA_MOSI (0 + 11)

#define SX126X_DIO2_AS_RF_SWITCH true
#define SX126X_DIO3_TCXO_VOLTAGE 1.8
#define SX126X_POWER_EN 37

// buttons
#define PIN_BUTTON1 (32 + 6)
#define BUTTON_PIN PIN_BUTTON1
#define PIN_USER_BTN BUTTON_PIN

// built-ins
#define VBAT_MV_PER_LSB (0.73242188F) // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096

#define VBAT_DIVIDER (0.5F)      // 150K + 150K voltage divider on VBAT, actually 100K + 100K
#define VBAT_DIVIDER_COMP (2.0F) // Compensation factor for the VBAT divider

#define PIN_VBAT_READ (0 + 2)
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

class NanoG2Ultra : public mesh::MainBoard
{
protected:
  uint8_t startup_reason;

public:
  void begin();
  void loop();
  uint16_t getBattMilliVolts() override;
  bool startOTAUpdate(const char *id, char reply[]) override;

  uint8_t getStartupReason() const override
  {
    return startup_reason;
  }

  const char *getManufacturerName() const override
  {
    return "Nano G2 Ultra";
  }

  void reboot() override
  {
    NVIC_SystemReset();
  }
};
