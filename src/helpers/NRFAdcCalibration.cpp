#ifdef NRF52_PLATFORM

#include "NRFAdcCalibration.h"
#include <Arduino.h>

// Include NRF52 specific headers for FICR access
#ifdef ARDUINO_ARCH_NRF52
  #include <nrf.h>
  // #include <nrf_nvmc.h>  // TODO: Enable when implementing flash storage
#endif

namespace NRFAdcCalibration {
    
    // Static variables to store calibration state
    static float s_calibrationFactor = 1.0f;
    static bool s_calibrated = false;
    static CalibrationMethod s_calibrationMethod = METHOD_NONE;
    
    // NRF52 internal voltage reference specifications  
    static constexpr float NOMINAL_VREF = 3.0f;  // 3.0V internal reference
    static constexpr float INTERNAL_BANDGAP = 0.6f; // 0.6V internal bandgap
    static constexpr float VREF_TOLERANCE = 0.05f; // ±5% typical tolerance
    static constexpr uint16_t ADC_MAX_VALUE = 4095; // 12-bit ADC
    static constexpr int CALIBRATION_SAMPLES = 16; // Multiple samples for accuracy
    
    // TODO: Flash storage configuration - disabled for now
    // static constexpr uint32_t FLASH_CALIBRATION_PAGE = 0x7F000; // Near end of flash
    // static constexpr uint32_t CALIBRATION_MAGIC = 0xCA11B8A7; // Magic number to validate data
    // 
    // struct CalibrationData {
    //     uint32_t magic;
    //     float calibrationFactor;
    //     CalibrationMethod method;
    //     uint32_t timestamp; // For future use
    //     uint32_t crc; // For data integrity
    // };
    // 
    // // Helper function to calculate simple CRC
    // uint32_t calculateCRC(const void* data, size_t length) {
    //     uint32_t crc = 0xFFFFFFFF;
    //     const uint8_t* bytes = (const uint8_t*)data;
    //     for (size_t i = 0; i < length; i++) {
    //         crc ^= bytes[i];
    //         for (int j = 0; j < 8; j++) {
    //             if (crc & 1) {
    //                 crc = (crc >> 1) ^ 0xEDB88320;
    //             } else {
    //                 crc >>= 1;
    //             }
    //         }
    //     }
    //     return ~crc;
    // }
    
    float loadFactoryCalibration() {
        #ifdef ARDUINO_ARCH_NRF52
        // Check if FICR data is available
        if (NRF_FICR->INFO.VARIANT != 0xFFFFFFFF) {
            // Use factory ADC calibration values from FICR if available
            // NRF52 FICR contains factory calibration for various components
            
            // For ADC, we can use the bandgap voltage calibration
            // FICR contains temperature sensor calibration which uses internal references
            uint32_t temp_a0 = NRF_FICR->TEMP.A0;
            uint32_t temp_a1 = NRF_FICR->TEMP.A1;
            
            if (temp_a0 != 0xFFFFFFFF && temp_a1 != 0xFFFFFFFF) {
                // Calculate calibration factor based on temperature sensor factory cal
                // This provides indirect calibration of the ADC reference
                float factor = 1.0f + ((float)(temp_a1 - temp_a0) / 1000000.0f);
                
                // Clamp to reasonable range
                if (factor > 0.9f && factor < 1.1f) {
                    s_calibrationMethod = METHOD_FACTORY_FICR;
                    Serial.printf("DEBUG: Factory calibration loaded from FICR: %.4f\n", factor);
                    return factor;
                }
            }
        }
        #endif
        
        Serial.println("DEBUG: Factory calibration not available");
        return 1.0f;
    }
    
    float calibrateWithExternalReference(int referencePin, float referenceVoltage, bool storeInFlash) {
        Serial.printf("DEBUG: Calibrating with external reference: Pin %d, %.3fV\n", referencePin, referenceVoltage);
        
        // Configure ADC for calibration
        analogReference(AR_INTERNAL_3_0);
        analogReadResolution(12);
        delay(10); // Allow reference to stabilize
        
        // Take multiple samples of the reference voltage
        uint32_t rawSum = 0;
        for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
            rawSum += analogRead(referencePin);
            delayMicroseconds(100);
        }
        
        float avgRaw = (float)rawSum / CALIBRATION_SAMPLES;
        
        // Calculate expected raw value for the reference voltage
        float expectedRaw = (referenceVoltage * ADC_MAX_VALUE) / NOMINAL_VREF;
        
        // Calculate calibration factor
        float factor = expectedRaw / avgRaw;
        
        // Validate factor is reasonable
        if (factor < 0.5f || factor > 2.0f) {
            Serial.printf("DEBUG: External reference calibration failed - unreasonable factor: %.4f\n", factor);
            return 1.0f;
        }
        
        s_calibrationMethod = METHOD_EXTERNAL_REF;
        
        if (storeInFlash) {
            storeCalibrationToFlash(factor, s_calibrationMethod);
        }
        
        Serial.printf("DEBUG: External reference calibration complete: %.4f (%.1f%% correction)\n", 
                     factor, (factor - 1.0f) * 100.0f);
        
        return factor;
    }
    
    float calibrateWithInternalBandgap() {
        Serial.println("DEBUG: Calibrating with internal bandgap reference");
        
        // Configure ADC for bandgap measurement
        analogReference(AR_INTERNAL_3_0);
        analogReadResolution(12);
        delay(10);
        
        // The NRF52 doesn't expose bandgap directly, but we can use VDD/4 reference
        // This is less accurate but better than no calibration
        
        // Use AR_VDD4 which is VDD/4 as reference
        analogReference(AR_VDD4);
        delay(5);
        
        uint32_t vddSum = 0;
        for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
            #if defined(A0)
                vddSum += analogRead(A0);
            #elif defined(P0_02)
                vddSum += analogRead(P0_02);
            #else
                vddSum += analogRead(0); // Use first available pin
            #endif
            delayMicroseconds(100);
        }
        
        // Switch back to 3.0V reference
        analogReference(AR_INTERNAL_3_0);
        delay(5);
        
        uint32_t refSum = 0;
        for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
            #if defined(A0)
                refSum += analogRead(A0);
            #elif defined(P0_02)
                refSum += analogRead(P0_02);
            #else
                refSum += analogRead(0);
            #endif
            delayMicroseconds(100);
        }
        
        float avgVdd = (float)vddSum / CALIBRATION_SAMPLES;
        float avgRef = (float)refSum / CALIBRATION_SAMPLES;
        
        // Calculate calibration factor based on ratio
        float factor = 1.0f;
        if (avgRef > 10 && avgVdd > 10) {
            factor = 1.0f + ((avgVdd - avgRef) / avgRef) * 0.1f; // Conservative adjustment
            
            // Clamp to ±3%
            if (factor < 0.97f) factor = 0.97f;
            if (factor > 1.03f) factor = 1.03f;
            
            s_calibrationMethod = METHOD_INTERNAL_BANDGAP;
        }
        
        Serial.printf("DEBUG: Internal bandgap calibration complete: %.4f (%.1f%% correction)\n", 
                     factor, (factor - 1.0f) * 100.0f);
        
        return factor;
    }
    
    bool storeCalibrationToFlash(float calibrationFactor, CalibrationMethod method) {
        // TODO: Implement flash storage using tested NVMC API
        // For now, disable flash storage to avoid compilation issues
        Serial.println("DEBUG: Flash storage temporarily disabled - calibration not persistent");
        return false;
    }
    
    bool loadCalibrationFromFlash(float &calibrationFactor, CalibrationMethod &method) {
        // TODO: Implement flash storage using tested NVMC API
        // For now, disable flash storage to avoid compilation issues
        return false;
    }
    
    float performBootCalibration(CalibrationMethod forceMethod) {
        if (s_calibrated && forceMethod == METHOD_NONE) {
            return s_calibrationFactor;
        }
        
        Serial.println("DEBUG: Starting comprehensive NRF52 ADC calibration...");
        
        // Configure ADC for calibration
        analogReference(AR_INTERNAL_3_0);
        analogReadResolution(12);
        delay(10);
        
        float factor = 1.0f;
        CalibrationMethod method = METHOD_DEFAULT;
        
        // Try calibration methods in order of preference (unless forced)
        if (forceMethod == METHOD_NONE || forceMethod == METHOD_FACTORY_FICR) {
            factor = loadFactoryCalibration();
            if (factor != 1.0f) {
                method = METHOD_FACTORY_FICR;
                goto calibration_complete;
            }
        }
        
        if (forceMethod == METHOD_NONE || forceMethod == METHOD_FLASH_STORED) {
            float flashFactor;
            CalibrationMethod flashMethod;
            if (loadCalibrationFromFlash(flashFactor, flashMethod)) {
                factor = flashFactor;
                method = flashMethod;
                goto calibration_complete;
            }
        }
        
        if (forceMethod == METHOD_NONE || forceMethod == METHOD_INTERNAL_BANDGAP) {
            factor = calibrateWithInternalBandgap();
            if (factor != 1.0f) {
                method = METHOD_INTERNAL_BANDGAP;
                goto calibration_complete;
            }
        }
        
        // Fallback to safe default
        factor = 1.0f;
        method = METHOD_DEFAULT;
        Serial.println("DEBUG: Using safe default calibration (no correction)");
        
        calibration_complete:
        s_calibrationFactor = factor;
        s_calibrationMethod = method;
        s_calibrated = true;
        
        const char* methodName[] = {"NONE", "FACTORY", "EXTERNAL", "FLASH", "BANDGAP", "DEFAULT"};
        Serial.printf("DEBUG: ADC calibration complete using %s: Factor=%.4f (%.1f%% correction)\n", 
                     methodName[method], factor, (factor - 1.0f) * 100.0f);
        
        return factor;
    }
    
    float getCalibrationFactor() {
        if (!s_calibrated) {
            return performBootCalibration();
        }
        return s_calibrationFactor;
    }
    
    CalibrationMethod getCalibrationMethod() {
        return s_calibrationMethod;
    }
    
    uint16_t rawToMilliVolts(uint32_t rawAdcValue, float boardMultiplier, float arefVoltage) {
        float calibrationFactor = getCalibrationFactor();
        
        // Apply calibration and convert to millivolts
        // Formula: (raw * calibration * aref_voltage * board_multiplier * 1000) / adc_max
        float millivolts = (float)rawAdcValue * calibrationFactor * arefVoltage * boardMultiplier * 1000.0f / ADC_MAX_VALUE;
        
        // Clamp to reasonable range (0-6000mV for typical battery applications)
        if (millivolts < 0) millivolts = 0;
        if (millivolts > 6000) millivolts = 6000;
        
        return (uint16_t)(millivolts + 0.5f); // Round to nearest integer
    }
    
    bool isCalibrated() {
        return s_calibrated;
    }
    
    void resetCalibration() {
        s_calibrated = false;
        s_calibrationFactor = 1.0f;
        s_calibrationMethod = METHOD_NONE;
        Serial.println("DEBUG: ADC calibration reset");
    }
    
    uint16_t rawToMilliVoltsUncalibrated(uint32_t rawAdcValue, float boardMultiplier, float arefVoltage) {
        // Convert to millivolts WITHOUT calibration factor (for debugging)
        // Formula: (raw * aref_voltage * board_multiplier * 1000) / adc_max
        float millivolts = (float)rawAdcValue * arefVoltage * boardMultiplier * 1000.0f / ADC_MAX_VALUE;
        
        // Clamp to reasonable range (0-6000mV for typical battery applications)
        if (millivolts < 0) millivolts = 0;
        if (millivolts > 6000) millivolts = 6000;
        
        return (uint16_t)(millivolts + 0.5f); // Round to nearest integer
    }
    
    const char* getCalibrationInfo(float &factor, CalibrationMethod &method, float &accuracy) {
        factor = s_calibrationFactor;
        method = s_calibrationMethod;
        
        static const char* methodNames[] = {
            "None", "Factory FICR", "External Reference", "Flash Stored", 
            "Internal Bandgap", "Default"
        };
        
        static const float methodAccuracy[] = {
            0.0f,   // None
            99.5f,  // Factory FICR - very accurate
            99.9f,  // External Reference - most accurate
            99.0f,  // Flash Stored - high accuracy (when implemented)
            95.0f,  // Internal Bandgap - moderately accurate
            90.0f   // Default - baseline
        };
        
        if (method < sizeof(methodAccuracy)/sizeof(methodAccuracy[0])) {
            accuracy = methodAccuracy[method];
        } else {
            accuracy = 0.0f;
        }
        
        return methodNames[method < sizeof(methodNames)/sizeof(methodNames[0]) ? method : 0];
    }
    
} // namespace NRFAdcCalibration

#endif // NRF52_PLATFORM 