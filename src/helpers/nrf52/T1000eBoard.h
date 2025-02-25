#pragma once

#include <MeshCore.h>
#include <Arduino.h>

#define HAS_T1000e_POWEROFF

// LoRa and SPI pins
#define  P_LORA_DIO_1   (32 + 1)  // P1.1
#define  P_LORA_NSS     (0 + 12)  // P0.12
#define  P_LORA_RESET   (32 + 10) // P1.10
#define  P_LORA_BUSY    (0 + 7)   // P0.7
#define  P_LORA_SCLK    (0 + 11)  // P0.11
#define  P_LORA_MISO    (32 + 8)  // P1.8
#define  P_LORA_MOSI    (32 + 9)  // P0.9
#define  LR1110_POWER_EN  37
 
#define LR11X0_DIO_AS_RF_SWITCH  true
#define LR11X0_DIO3_TCXO_VOLTAGE   1.6

// built-ins
//#define  PIN_VBAT_READ    5
//#define  ADC_MULTIPLIER   (3 * 1.73 * 1000)

class T1000eBoard : public mesh::MainBoard {
protected:
  uint8_t startup_reason;
  uint8_t btn_prev_state;

public:
  void begin() {
    // for future use, sub-classes SHOULD call this from their begin()
    startup_reason = BD_STARTUP_NORMAL;
    btn_prev_state = HIGH;

    pinMode(BATTERY_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);

// Doesn't seem to be a pwr en pin ...
//    pinMode(LR1110_POWER_EN, OUTPUT);
//    digitalWrite(LR1110_POWER_EN, HIGH);
    delay(10);   // give sx1262 some time to power up
  }
  void begin();

  uint16_t getBattMilliVolts() override {
    analogReadResolution(12);
    float volts = (analogRead(BATTERY_PIN) * ADC_MULTIPLIER * AREF_VOLTAGE) / 4096;
    return volts * 1000;
  }

  uint8_t getStartupReason() const override { return startup_reason; }

  const char* getManufacturerName() const override {
    return "Seeed Tracker T1000-e";
  }

  int buttonStateChanged() {
    uint8_t v = digitalRead(BUTTON_PIN);
    if (v != btn_prev_state) {
      btn_prev_state = v;
      return (v == LOW) ? 1 : -1;
    }
    return 0;
  }

  void powerOff() {
    #ifdef GNSS_AIROHA
        digitalWrite(GPS_VRTC_EN, LOW);
        digitalWrite(PIN_GPS_RESET, LOW);
        digitalWrite(GPS_SLEEP_INT, LOW);
        digitalWrite(GPS_RTC_INT, LOW);
        pinMode(GPS_RESETB_OUT, OUTPUT);
        digitalWrite(GPS_RESETB_OUT, LOW);
    #endif
    
    #ifdef BUZZER_EN_PIN
        digitalWrite(BUZZER_EN_PIN, LOW);
    #endif
    
    #ifdef PIN_3V3_EN
        digitalWrite(PIN_3V3_EN, LOW);
    #endif

    digitalWrite(LED_PIN, LOW);
    nrf_gpio_cfg_sense_input(digitalPinToInterrupt(BUTTON_PIN), NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);
    sd_power_system_off();
  }

  void reboot() override {
    NVIC_SystemReset();
  }

//  bool startOTAUpdate() override;
};