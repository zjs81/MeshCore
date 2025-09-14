#include "NRF_SLEEP.h"

#if defined(NRF52_PLATFORM)
extern "C" {
#include <nrf_error.h>
}

volatile bool NRF_SLEEP::s_initialized = false;
NRF_SLEEP::Role NRF_SLEEP::s_role = NRF_SLEEP::Role::Repeater;
uint32_t NRF_SLEEP::s_bleIdleStopMs = 0;
uint32_t NRF_SLEEP::s_lastBleActivityMs = 0;
uint32_t NRF_SLEEP::s_bleWindowOnMs = 0;
uint32_t NRF_SLEEP::s_bleWindowPeriodMs = 0;
uint32_t NRF_SLEEP::s_bleWindowStartMs = 0;

static inline uint32_t ms_now() { return millis(); }

void NRF_SLEEP::begin(Role role,
                      uint32_t bleIdleStopSecs,
                      bool enableDcdc,
                      bool disableUsbSerial) {
  s_role = role;
  s_bleIdleStopMs = (bleIdleStopSecs > 0) ? (bleIdleStopSecs * 1000UL) : 0UL;
  s_lastBleActivityMs = ms_now();
  s_bleWindowOnMs = 0;
  s_bleWindowPeriodMs = 0;
  s_bleWindowStartMs = 0;

  configurePower(enableDcdc, disableUsbSerial);

  // Ensure BLE advertising state is known at startup
#if NRF_SLEEP_HAS_BLUEFRUIT
  if (s_role == Role::BLECompanion) {
    // Start with ADV active if user code already began BLE, otherwise do nothing.
    // We won't call Bluefruit.begin() here to avoid ownership issues.
    if (Bluefruit.Advertising.isRunning()) {
      s_lastBleActivityMs = ms_now();
    }
  } else {
    // Repeater role — prefer BLE off by default if running
    if (Bluefruit.Advertising.isRunning()) {
      Bluefruit.Advertising.stop();
    }
  }
#endif

  s_initialized = true;
}

void NRF_SLEEP::loop(bool /*radioAlwaysOn*/) {
  if (!s_initialized) return;
  manageBLE();
  cpuIdleWait();
}

void NRF_SLEEP::notifyBLEActivity() {
  s_lastBleActivityMs = ms_now();
}

void NRF_SLEEP::setBLEAdvertisingWindows(uint32_t windowOnMs, uint32_t periodMs) {
  if (windowOnMs == 0 || periodMs == 0 || windowOnMs >= periodMs) {
    s_bleWindowOnMs = 0;
    s_bleWindowPeriodMs = 0;
    s_bleWindowStartMs = 0;
    return;
  }
  s_bleWindowOnMs = windowOnMs;
  s_bleWindowPeriodMs = periodMs;
  s_bleWindowStartMs = ms_now();
}

void NRF_SLEEP::configurePower(bool enableDcdc, bool disableUsbSerial) {
  if (enableDcdc) {
    // Enable DCDC for both REG0 and REG1 if available; guard with sd_ calls when SoftDevice is present.
    // Safe to call even if already set.
    (void) sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
  }

  // Prefer low power mode
  (void) sd_power_mode_set(NRF_POWER_MODE_LOWPWR);

  // Optionally disable USB CDC to allow better idle currents
  if (disableUsbSerial) {
#if defined(USBCON)
    if (Serial) {
      Serial.end();
    }
#endif
  }
}

void NRF_SLEEP::manageBLE() {
#if NRF_SLEEP_HAS_BLUEFRUIT
  // If BLE hasn't been begun by app code, do nothing.
  // Use a conservative probe: querying isRunning() is safe and returns false when not begun.

  const bool advRunning = Bluefruit.Advertising.isRunning();

  // Repeater: keep BLE off
  if (s_role == Role::Repeater) {
    if (advRunning) Bluefruit.Advertising.stop();
    return;
  }

  // BLE Companion behavior
  const uint32_t now = ms_now();

  // Optional advertising windowing: on for s_bleWindowOnMs in each period
  if (s_bleWindowPeriodMs > 0) {
    const uint32_t elapsed = now - s_bleWindowStartMs;
    const uint32_t inPeriod = elapsed % s_bleWindowPeriodMs;
    const bool shouldAdv = (inPeriod < s_bleWindowOnMs);
    if (shouldAdv) {
      if (!advRunning) {
        // Start advertising if BLE stack is ready
        Bluefruit.Advertising.start(0);
        s_lastBleActivityMs = now;
      }
    } else {
      if (advRunning) Bluefruit.Advertising.stop();
    }
    return; // windowing manages adv state; skip idle-stop logic below
  }

  // Idle-stop logic: stop advertising after inactivity timeout if not connected
  if (s_bleIdleStopMs > 0) {
    if (!Bluefruit.connected()) {
      if (advRunning && (now - s_lastBleActivityMs) >= s_bleIdleStopMs) {
        Bluefruit.Advertising.stop();
      }
    } else {
      // Connected: refresh activity timer
      s_lastBleActivityMs = now;
    }
  }
#endif
}

void NRF_SLEEP::cpuIdleWait() {
  // Put CPU to idle until next IRQ (System ON sleep). This allows LoRa RX IRQs,
  // RTC, GPIO, etc., to wake immediately. Avoids busy-wait loops.
  uint32_t err = sd_app_evt_wait();
  if (err == NRF_ERROR_SOFTDEVICE_NOT_ENABLED) {
    // Fallback when SoftDevice isn't enabled: standard event-based sleep.
    __SEV();
    __WFE();
    __WFE();
  }
}

#endif // NRF52_PLATFORM


