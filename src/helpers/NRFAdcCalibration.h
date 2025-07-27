#pragma once

#ifdef NRF52_PLATFORM

#include <Arduino.h>

namespace NRFAdcCalibration {
    
    // Calibration methods (in order of preference)
    enum CalibrationMethod {
        METHOD_NONE = 0,
        METHOD_FACTORY_FICR = 1,     // Use factory calibration from FICR registers
        METHOD_EXTERNAL_REF = 2,     // Use known external voltage reference
        METHOD_FLASH_STORED = 3,     // Use calibration stored in flash
        METHOD_INTERNAL_BANDGAP = 4, // Use internal 0.6V bandgap reference
        METHOD_DEFAULT = 5           // Safe default (1.0 factor)
    };
    
    /**
     * Performs comprehensive ADC calibration using the best available method.
     * Order of preference:
     * 1. Factory calibration from FICR registers
     * 2. External voltage reference (if configured)
     * 3. User calibration stored in flash
     * 4. Internal bandgap reference calibration
     * 5. Safe default (no correction)
     * 
     * @param forceMethod Force a specific calibration method (optional)
     * @return calibration factor to multiply raw ADC readings by
     */
    float performBootCalibration(CalibrationMethod forceMethod = METHOD_NONE);
    
    /**
     * Calibrate using a known external voltage reference.
     * This is the most accurate method for production use.
     * 
     * @param referencePin Analog pin connected to precision voltage reference
     * @param referenceVoltage Known voltage of the reference (in volts)
     * @param storeInFlash Save calibration to flash for future use
     * @return calibration factor
     */
    float calibrateWithExternalReference(int referencePin, float referenceVoltage, bool storeInFlash = true);
    
    /**
     * Load factory calibration values from NRF52 FICR registers.
     * These are programmed during chip manufacturing and provide good accuracy.
     * 
     * @return calibration factor from factory data, or 1.0 if unavailable
     */
    float loadFactoryCalibration();
    
    /**
     * Calibrate using internal 0.6V bandgap reference.
     * Less accurate than external reference but better than default.
     * 
     * @return calibration factor from bandgap measurement
     */
    float calibrateWithInternalBandgap();
    
    /**
     * Store calibration data to flash memory for persistence.
     * 
     * @param calibrationFactor The factor to store
     * @param method The calibration method used
     * @return true if successful
     */
    bool storeCalibrationToFlash(float calibrationFactor, CalibrationMethod method);
    
    /**
     * Load calibration data from flash memory.
     * 
     * @param calibrationFactor Output parameter for loaded factor
     * @param method Output parameter for calibration method
     * @return true if valid calibration was loaded
     */
    bool loadCalibrationFromFlash(float &calibrationFactor, CalibrationMethod &method);
    
    /**
     * Get the cached calibration factor from the boot calibration.
     * If calibration hasn't been performed yet, triggers auto-calibration.
     * 
     * @return calibration factor for ADC readings
     */
    float getCalibrationFactor();
    
    /**
     * Get information about the current calibration method.
     * 
     * @return the method used for current calibration
     */
    CalibrationMethod getCalibrationMethod();
    
    /**
     * Convert raw ADC reading to millivolts using calibration and board-specific multiplier.
     * This combines the calibration factor with the board's voltage divider compensation.
     * 
     * @param rawAdcValue Raw 12-bit ADC reading (0-4095)
     * @param boardMultiplier Board-specific voltage divider multiplier 
     * @param arefVoltage ADC reference voltage in volts (typically 3.0V for AR_INTERNAL_3_0)
     * @return Battery voltage in millivolts
     */
    uint16_t rawToMilliVolts(uint32_t rawAdcValue, float boardMultiplier, float arefVoltage = 3.0f);
    
    /**
     * Check if ADC calibration has been performed.
     * 
     * @return true if calibration is available, false otherwise
     */
    bool isCalibrated();
    
    /**
     * Reset calibration (for testing purposes).
     * Forces recalibration on next performBootCalibration() call.
     */
    void resetCalibration();
    
    /**
     * Debug function to get raw ADC voltage without calibration factor applied.
     * Useful for diagnosing calibration issues.
     * 
     * @param rawAdcValue Raw 12-bit ADC reading (0-4095)
     * @param boardMultiplier Board-specific voltage divider multiplier 
     * @param arefVoltage ADC reference voltage in volts (typically 3.0V for AR_INTERNAL_3_0)
     * @return Uncalibrated voltage in millivolts
     */
    uint16_t rawToMilliVoltsUncalibrated(uint32_t rawAdcValue, float boardMultiplier, float arefVoltage = 3.0f);
    
    /**
     * Get detailed calibration information for debugging.
     * 
     * @param factor Output calibration factor
     * @param method Output calibration method
     * @param accuracy Output estimated accuracy percentage
     * @return descriptive string of calibration status
     */
    const char* getCalibrationInfo(float &factor, CalibrationMethod &method, float &accuracy);
}

#endif // NRF52_PLATFORM 