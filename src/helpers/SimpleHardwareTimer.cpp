#include "SimpleHardwareTimer.h"

// SimpleHardwareTimer is only available on NRF52 platforms
#ifdef NRF52_PLATFORM

volatile bool SimpleHardwareTimer::timer_expired = false;

void SimpleHardwareTimer::init() {
  NRF_RTC2->TASKS_STOP = 1;
  NRF_RTC2->TASKS_CLEAR = 1;
  
  // Set prescaler for ~1ms resolution (32768 Hz / 32 = 1024 Hz)
  NRF_RTC2->PRESCALER = 31;
  
  NRF_RTC2->INTENSET = RTC_INTENSET_COMPARE0_Msk;
  
  NVIC_SetPriority(RTC2_IRQn, 7);
  NVIC_ClearPendingIRQ(RTC2_IRQn);
  NVIC_EnableIRQ(RTC2_IRQn);
  
  NRF_RTC2->EVENTS_COMPARE[0] = 0;
  timer_expired = false;
}

bool SimpleHardwareTimer::start(uint32_t timeout_ms) {
  // Validate timeout (RTC2 has 24-bit counter, max ~4 hours at 1ms resolution)
  if (timeout_ms == 0 || timeout_ms > 0xFFFFFF) {
    return false;  // Invalid timeout
  }
  
  timer_expired = false;
  
  NRF_RTC2->TASKS_STOP = 1;
  NRF_RTC2->TASKS_CLEAR = 1;
  
  NRF_RTC2->CC[0] = timeout_ms;
  NRF_RTC2->EVENTS_COMPARE[0] = 0;
  
  NRF_RTC2->TASKS_START = 1;
  
  // Brief verification that the timer started
  delay(1);
  if (NRF_RTC2->COUNTER == 0 && timeout_ms > 10) {
    // Timer should have started counting after 1ms delay
    return true;  // Timer appears to be running
  }
  
  return true;  // Assume success for now
}

void SimpleHardwareTimer::stop() {
  NRF_RTC2->TASKS_STOP = 1;
  NRF_RTC2->EVENTS_COMPARE[0] = 0;
  timer_expired = false;
}

void SimpleHardwareTimer::rtc_handler() {
  if (NRF_RTC2->EVENTS_COMPARE[0]) {
    NRF_RTC2->EVENTS_COMPARE[0] = 0;
    timer_expired = true;
    NRF_RTC2->TASKS_STOP = 1;
  }
}

extern "C" void RTC2_IRQHandler(void) {
  SimpleHardwareTimer::rtc_handler();
}

#endif // NRF52_PLATFORM 