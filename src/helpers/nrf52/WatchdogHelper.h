#pragma once

#ifdef NRF52_PLATFORM

#include <Arduino.h>
#include <nrf_wdt.h>

// Simple watchdog helper for NRF52 to prevent complete system hangs
class WatchdogHelper {
private:
    static bool _enabled;
    static uint32_t _timeout_ms;

public:
    // Initialize watchdog with timeout in milliseconds (default 30 seconds)
    static void begin(uint32_t timeout_ms = 30000) {
        if (_enabled) return;  // Already initialized
        
        _timeout_ms = timeout_ms;
        
        // Configure WDT
        // Timeout in seconds = (CRV + 1) / 32768
        // So for timeout_ms milliseconds: CRV = (timeout_ms * 32768 / 1000) - 1
        uint32_t crv = (timeout_ms * 32768UL / 1000UL) - 1;
        
        NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos) |
                          (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos);
        NRF_WDT->CRV = crv;
        NRF_WDT->RREN = WDT_RREN_RR0_Enabled << WDT_RREN_RR0_Pos;
        NRF_WDT->TASKS_START = 1;
        
        _enabled = true;
        
        #if defined(MESH_DEBUG)
        Serial.printf("Watchdog enabled: %d ms timeout\n", timeout_ms);
        #endif
    }
    
    // Feed the watchdog to prevent reset
    static inline void feed() {
        if (_enabled) {
            NRF_WDT->RR[0] = WDT_RR_RR_Reload;
        }
    }
    
    // Check if watchdog is enabled
    static bool isEnabled() {
        return _enabled;
    }
};

#endif // NRF52_PLATFORM

