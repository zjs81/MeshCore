#pragma once

#include <MeshCore.h>
#include <Arduino.h>

#ifdef XIAO_NRF52

class IkokaStickNRFBoard : public mesh::MainBoard {
protected:
  uint8_t startup_reason;

public:
  void begin();
  uint8_t getStartupReason() const override { return startup_reason; }

#if defined(P_LORA_TX_LED)
  void onBeforeTransmit() override {
    digitalWrite(P_LORA_TX_LED, LOW);   // turn TX LED on
    #if defined(LED_BLUE)
       // turn off that annoying blue LED before transmitting
       digitalWrite(LED_BLUE, HIGH);
    #endif
  }
  void onAfterTransmit() override {
    digitalWrite(P_LORA_TX_LED, HIGH);   // turn TX LED off
    #if defined(LED_BLUE)
       // do it after transmitting too, just in case
       digitalWrite(LED_BLUE, HIGH);
    #endif
  }
#endif

  uint16_t getBattMilliVolts() override {
    // Please read befor going further ;)
    // https://wiki.seeedstudio.com/XIAO_BLE#q3-what-are-the-considerations-when-using-xiao-nrf52840-sense-for-battery-charging

    // We can't drive VBAT_ENABLE to HIGH as long 
    // as we don't know wether we are charging or not ...
    // this is a 3mA loss (4/1500)
    digitalWrite(VBAT_ENABLE, LOW);
    int adcvalue = 0;
    analogReadResolution(12);
    analogReference(AR_INTERNAL_3_0);  
    delay(10);
    adcvalue = analogRead(PIN_VBAT);
    return (adcvalue * ADC_MULTIPLIER * AREF_VOLTAGE) / 4.096;
  }

  const char *getManufacturerName() const override {
    return MANUFACTURER_STRING;
  }

  void reboot() override {
    NVIC_SystemReset();
  }

  bool startOTAUpdate(const char *id, char reply[]) override;
};

#endif
