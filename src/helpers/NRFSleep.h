#pragma once

#include <Arduino.h>

// NRFSleep is only available on NRF52 platforms
#ifdef NRF52_PLATFORM

#include <nrf_gpio.h>
#include <bluefruit.h>
#ifdef PIN_BUZZER
#include <NonBlockingRtttl.h>
#endif

// Forward declarations
namespace mesh { class Radio; }

// Pin definitions - these will be defined in variant.h
#ifndef LORA_DIO_1
#define LORA_DIO_1 (33)  // Default for T1000-E
#endif
#ifndef LORA_BUSY  
#define LORA_BUSY (7)    // Default for T1000-E
#endif

// Forward declaration - will be included in .cpp
class RadioLibWrapper;

/**
 * NRF52 Sleep Management Helper
 * Implements CLIENT mode power management like Meshtastic:
 * - Uses sd_power_system_off() for deep sleep (0.5μA)
 * - No Bluetooth during deep sleep
 * - Wake via button/LoRa interrupts
 * - LoRa activity tracking prevents premature sleep
 */
class NRFSleep {
private:
  static volatile uint32_t last_lora_activity;
  static uint32_t last_sleep_check;
  static mesh::Radio* radio_instance;
  
  // Hybrid sleep cycle management
  static uint8_t sleep_cycle_count;
  static const uint8_t LIGHT_SLEEP_CYCLES = 3; // Light sleep cycles before deep sleep
  
  // LoRa timing calculation  
  static uint32_t calculateMaxLoRaTransactionTime();
  static uint32_t getLoRaSleepDelayMs();
  
  // LoRa DIO1 interrupt handler - captures TX/RX/Timeout events
  static void lora_dio1_interrupt();
  
  // RTC wake-up timer setup for system off mode
  static void setupRTCWake(uint32_t seconds);
  
public:
  // Initialize sleep management system
  static void init();
  static void init(mesh::Radio* radio);
  
  // Check if sleep should be delayed due to recent LoRa activity
  static bool shouldDelayForLora();
  
  // Hybrid sleep strategy - balances connectivity and power
  static void enterHybridSleep(uint32_t sleep_duration_ms, bool ble_connected);
  
  // Enter deep sleep using CLIENT mode (sd_power_system_off)
  static void enterDeepSleep();
  
  // Enter light sleep with wake sources (for short delays)
  static void enterLightSleep(uint32_t timeout_ms);
  
  // Update LoRa activity timestamp (can be called externally)
  static void markLoraActivity();
  
  // Power down T1000E peripherals for deep sleep (CLIENT mode)
  static void powerDownPeripherals();
  
  // High-level sleep management - handles all sleep logic
  static void attemptSleep();
  
  // Complete sleep loop management for NRF boards
  static void manageSleepLoop();
};

#endif // NRF52_PLATFORM 