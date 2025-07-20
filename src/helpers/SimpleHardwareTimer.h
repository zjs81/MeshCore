#pragma once

#include <Arduino.h>

// Only include NRF headers on NRF platforms
#ifdef NRF52_PLATFORM
#include <nrf.h>
#endif

/**
 * Hardware timer using RTC2 for precise timeout functionality.
 * Provides 1ms resolution with interrupt-driven operation for low power.
 */
class SimpleHardwareTimer {
private:
  static volatile bool timer_expired;
  
public:
  static void init();
  static void start(uint32_t timeout_ms);
  static void stop();
  static bool isExpired() { return timer_expired; }
  static void rtc_handler();
}; 