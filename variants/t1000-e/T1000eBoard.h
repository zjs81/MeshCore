#pragma once

#include <MeshCore.h>
#include <Arduino.h>
#include <helpers/NRFAdcCalibration.h>
#include <nrf_gpio.h>
#ifdef PIN_BUZZER
#include <NonBlockingRtttl.h>
#endif
#include <bluefruit.h>

// Define button pressed state if not already defined by platformio.ini
#ifndef USER_BTN_PRESSED
#define USER_BTN_PRESSED LOW  // Default for most boards
#endif

// LoRa and SPI pins
#define  P_LORA_DIO_1   (32 + 1)  // P1.1
#define  P_LORA_NSS     (0 + 12)  // P0.12
#define  P_LORA_RESET   (32 + 10) // P1.10
#define  P_LORA_BUSY    (0 + 7)   // P0.7
#define  P_LORA_SCLK    (0 + 11)  // P0.11
#define  P_LORA_MISO    (32 + 8)  // P1.8
#define  P_LORA_MOSI    (32 + 9)  // P0.9

#define LR11X0_DIO_AS_RF_SWITCH  true
#define LR11X0_DIO3_TCXO_VOLTAGE   1.6

// built-ins
//#define  PIN_VBAT_READ    5
//#define  ADC_MULTIPLIER   (3 * 1.73 * 1000)

class T1000eBoard : public mesh::MainBoard {
protected:
  uint8_t startup_reason;
  uint8_t btn_prev_state;
  unsigned long btn_press_start;
  bool btn_long_press_triggered;
  bool power_button_enabled;

public:
  void begin();
  void loop(); // Power management

  // Battery voltage reading
  uint16_t getBattMilliVolts() override {
  #ifdef BATTERY_PIN
    #ifdef PIN_3V3_EN
      digitalWrite(PIN_3V3_EN, HIGH);
    #endif
    analogReference(AR_INTERNAL_3_0);
    analogReadResolution(12);
    delay(10);
    
    uint32_t raw = 0;
    for (int i = 0; i < 8; i++) {
      raw += analogRead(BATTERY_PIN);
    }
    raw = raw / 8;
    
    #ifdef PIN_3V3_EN
      digitalWrite(PIN_3V3_EN, LOW);
    #endif

    analogReference(AR_DEFAULT);  // put back to default
    analogReadResolution(10);

    // DEBUG: The board multiplier calculation was incorrect!
    // Original formula: (analogRead * ADC_MULTIPLIER * AREF_VOLTAGE) / 4096 * 1000
    // But ADC_MULTIPLIER should be applied to the final voltage, not divided by 4096
    // Correct formula: (raw * AREF_VOLTAGE / 4096) * ADC_MULTIPLIER * 1000 / 1000
    // Simplified: (raw * AREF_VOLTAGE * ADC_MULTIPLIER) / 4096
    
    const float boardMultiplier = ADC_MULTIPLIER; // This should just be the voltage divider multiplier
    uint16_t voltage = NRFAdcCalibration::rawToMilliVolts(raw, boardMultiplier, AREF_VOLTAGE);
    
    // Debug output (commented out for production)
    // Serial.printf("DEBUG T1000E Battery - Raw: %d, Multiplier: %.3f, Voltage: %dmV\n", 
    //               raw, boardMultiplier, voltage);
    
    return voltage;
  #else
    return 0;
  #endif
  }

  uint8_t getStartupReason() const override { return startup_reason; }
  
  const char* getManufacturerName() const override { 
    return "Seeed Tracker T1000-e";
   }

  // Button state tracking with long-press power off detection
  int buttonStateChanged() {
  #ifdef BUTTON_PIN
    // Allow disabling power button functionality for testing
    if (!power_button_enabled) {
      return 0;
    }
    
    uint8_t v = digitalRead(BUTTON_PIN);
    unsigned long now = millis();
    
    // Add debounce delay and safety checks
    static unsigned long last_button_check = 0;
    if (now - last_button_check < 50) {
      return 0; // Debounce - don't check too frequently
    }
    
    // Detect potential wake-from-sleep scenarios and reset stale timing
    static unsigned long last_awake_check = 0;
    if (last_awake_check > 0 && (now - last_awake_check) > 5000) {
      // System was likely asleep for >5 seconds - reset button timing to prevent false triggers
      if (btn_press_start > 0 && v != USER_BTN_PRESSED) {
        Serial.printf("DEBUG: Detected wake from sleep, resetting stale button timing (was %lu ms ago)\n", now - btn_press_start);
        btn_press_start = 0;
        btn_long_press_triggered = false;
      }
    }
    last_awake_check = now;
    last_button_check = now;
    
    // Track button press timing for long-press detection
    if (v == USER_BTN_PRESSED && btn_prev_state != USER_BTN_PRESSED) {
      // Button just pressed - start timing, but add safety delay
      btn_press_start = now;
      btn_long_press_triggered = false;
      Serial.printf("DEBUG: Power button pressed at %lu ms\n", now);
    } else if (v == USER_BTN_PRESSED && btn_prev_state == USER_BTN_PRESSED && btn_press_start > 0) {
      // Button held down - check for long press (5 seconds)
      unsigned long hold_time = now - btn_press_start;
      
      // Safety check: if hold_time is unreasonably large, reset timing
      if (hold_time > 10000) {
        Serial.printf("DEBUG: Invalid hold time %lu ms (corrupted timing from sleep/wake) - resetting\n", hold_time);
        btn_press_start = 0;  // Reset completely instead of setting to now
        btn_long_press_triggered = false;
        return 0;  // Exit early to prevent false shutdown
      }
      
      // Safety check: only start shutdown logic after system has been running for at least 10 seconds
      static bool system_startup_complete = false;
      if (!system_startup_complete && millis() > 10000) {
        system_startup_complete = true;
      }
      
      if (!system_startup_complete) {
        return 0; // Don't allow shutdown during startup
      }
      
      // Provide visual feedback during long press
      if (hold_time >= 3000) {
        // After 3 seconds, start blinking LED to indicate power off is coming
        #ifdef LED_PIN
          digitalWrite(LED_PIN, (millis() / 200) % 2); // Fast blink
        #endif
        Serial.printf("DEBUG: Power button held for %lu ms...\n", hold_time);
      }
      
      if (!btn_long_press_triggered && hold_time >= 5000) {
        btn_long_press_triggered = true;
        Serial.println("Power button held for 5 seconds - shutting down...");
        
        // Final LED indication
        #ifdef LED_PIN
          digitalWrite(LED_PIN, HIGH);
          delay(50);
          digitalWrite(LED_PIN, LOW);
          delay(50);
          digitalWrite(LED_PIN, HIGH);
          delay(50);
          digitalWrite(LED_PIN, LOW);
        #endif
        
        delay(100); // Give serial time to print
        powerOff();
        return 0; // Won't actually return since powerOff() shuts down
      }
    } else if (v != USER_BTN_PRESSED && btn_prev_state == USER_BTN_PRESSED) {
      // Button released - turn off LED if it was blinking and reset timing
      #ifdef LED_PIN
        digitalWrite(LED_PIN, LOW);
      #endif
      btn_press_start = 0; // Reset timing
      btn_long_press_triggered = false; // Reset trigger flag
      Serial.printf("DEBUG: Power button released at %lu ms\n", now);
    } else if (v != USER_BTN_PRESSED && btn_prev_state != USER_BTN_PRESSED && btn_press_start > 0) {
      // Button not pressed but we have stale timing - reset it
      Serial.printf("DEBUG: Clearing stale button timing (was %lu ms ago)\n", now - btn_press_start);
      btn_press_start = 0;
      btn_long_press_triggered = false;
    }
    
    // Debug: Log and fix unexpected state combinations
    if (v == USER_BTN_PRESSED && btn_prev_state == USER_BTN_PRESSED && btn_press_start == 0) {
      Serial.printf("DEBUG: Button held but no start time - setting start time to %lu ms\n", now);
      btn_press_start = now;
      btn_long_press_triggered = false;
    }
    
    // Additional safety: If button is not pressed but we have active timing, clear it
    if (v != USER_BTN_PRESSED && btn_press_start > 0 && (now - btn_press_start) > 1000) {
      Serial.printf("DEBUG: Button released but timing still active after %lu ms - clearing\n", now - btn_press_start);
      btn_press_start = 0;
      btn_long_press_triggered = false;
    }
    
    // Normal button state change tracking
    if (v != btn_prev_state) {
      Serial.printf("DEBUG: Button state change: %s -> %s\n", 
                    btn_prev_state ? "HIGH" : "LOW", 
                    v ? "HIGH" : "LOW");
      btn_prev_state = v;
      return (v == USER_BTN_PRESSED) ? 1 : -1;
    }
  #endif
    return 0;
  }

  // System power control
  void powerOff() override {
    #ifdef HAS_GPS
      digitalWrite(GPS_VRTC_EN, LOW);
        digitalWrite(GPS_RESET, LOW);
      digitalWrite(GPS_SLEEP_INT, LOW);
        digitalWrite(GPS_RTC_INT, LOW);
      pinMode(GPS_RESETB, OUTPUT);
      digitalWrite(GPS_RESETB, LOW);
    #endif
    
    #ifdef BUZZER_EN
      digitalWrite(BUZZER_EN, LOW);
    #endif
    
    #ifdef PIN_3V3_EN
      digitalWrite(PIN_3V3_EN, LOW);
    #endif

    #ifdef LED_PIN
      digitalWrite(LED_PIN, LOW);
    #endif
    #ifdef BUTTON_PIN
      nrf_gpio_cfg_sense_input(
        digitalPinToInterrupt(BUTTON_PIN),
        NRF_GPIO_PIN_NOPULL,
        #if USER_BTN_PRESSED == HIGH
        NRF_GPIO_PIN_SENSE_HIGH
        #else
        NRF_GPIO_PIN_SENSE_LOW
        #endif
      );
    #endif
    sd_power_system_off();
  }

  void reboot() override {
    NVIC_SystemReset();
  }

  // Power button control
  void setPowerButtonEnabled(bool enabled) {
    power_button_enabled = enabled;
    Serial.printf("DEBUG: Power button shutdown %s\n", enabled ? "ENABLED" : "DISABLED");
  }

  //  bool startOTAUpdate(const char* id, char reply[]) override;

};