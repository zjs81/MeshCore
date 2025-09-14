#pragma once

#include <stdint.h>

#if defined(NRF52_PLATFORM)

#include <Arduino.h>
extern "C" {
#include <nrf_soc.h>
}

// Optional Bluefruit dependency for BLE power control
#if __has_include(<bluefruit.h>)
  #include <bluefruit.h>
  #define NRF_SLEEP_HAS_BLUEFRUIT 1
#else
  #define NRF_SLEEP_HAS_BLUEFRUIT 0
#endif

// A tiny, self-contained helper for nRF52 low-power behavior tuned for MeshCore
// roles. It keeps LoRa in continuous RX (no duty-cycle) while minimizing CPU,
// BLE, and peripheral power. Intended to be called from setup() and loop().
class NRF_SLEEP {
public:
  enum class Role : uint8_t {
    Repeater,
    BLECompanion
  };

  // Initialize low-power settings. Call from setup().
  // - role: device behavior profile
  // - bleIdleStopSecs: if > 0, stop BLE advertising after this many seconds
  //   of idle (only applied for BLECompanion and when Bluefruit is present)
  // - enableDcdc: enable DCDC regulator for better efficiency
  // - disableUsbSerial: optionally end USB Serial to allow lower idle current
  static void begin(Role role,
                    uint32_t bleIdleStopSecs = 60,
                    bool enableDcdc = true,
                    bool disableUsbSerial = false);

  // Periodic maintenance. Call once per main loop iteration.
  // Keeping radioAlwaysOn true documents our Class-C style receive.
  static void loop(bool radioAlwaysOn = true);

  // Optional: explicit BLE activity hint to defer idle stop timer.
  static void notifyBLEActivity();

  // Optional: duty-cycle BLE advertising windows (for on-demand pairing).
  // Set windowOnMs>0 and periodMs>windowOnMs to periodically enable ADV.
  static void setBLEAdvertisingWindows(uint32_t windowOnMs, uint32_t periodMs);

private:
  static void configurePower(bool enableDcdc, bool disableUsbSerial);
  static void manageBLE();
  static void cpuIdleWait();

  static volatile bool s_initialized;
  static Role s_role;

  // BLE controls
  static uint32_t s_bleIdleStopMs;     // 0 disables
  static uint32_t s_lastBleActivityMs;
  static uint32_t s_bleWindowOnMs;     // 0 disables windowing
  static uint32_t s_bleWindowPeriodMs; // 0 disables windowing
  static uint32_t s_bleWindowStartMs;  // when current ADV window started
};

#endif // NRF52_PLATFORM


