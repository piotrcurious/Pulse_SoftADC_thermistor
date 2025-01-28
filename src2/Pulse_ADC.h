// SoftADC.h
#ifndef SOFTADC_H
#define SOFTADC_H

#include <Arduino.h>

// Library configuration
#define MAX_CHANNELS 8
#define CHARGE_THRESHOLD 900    // ~4.4V with 5V reference
#define DISCHARGE_THRESHOLD 100 // ~0.5V with 5V reference
#define DEFAULT_PULSE_LENGTH 10
#define VCC 5.0                // Supply voltage in volts
#define ADC_RESOLUTION 1024.0  // 10-bit ADC
#define CAPACITOR_VALUE 1e-7   // 100nF capacitor value in Farads

// Channel data structure
typedef struct {
    uint8_t pin;
    uint32_t chargeCount;
    uint32_t dischargeCount;
    uint32_t chargeAverage;
    uint32_t dischargeAverage;
    bool charging;
    bool active;
    float lastVoltage;
    unsigned long lastTime;
} Channel;

// Core functions
void SoftADC_begin(uint8_t pins[], uint8_t numPins, uint16_t pulseLen = DEFAULT_PULSE_LENGTH);
void SoftADC_update(void);
uint32_t SoftADC_read(uint8_t channel);
void SoftADC_setActive(uint8_t channel, bool state);

// Measurement and calculation functions
float SoftADC_calculateResistance(uint8_t channel);
float SoftADC_calculateTemperature(uint8_t channel);

// Advanced configuration functions (optional, can be added as needed)
void SoftADC_setCapacitorValue(float capacitance);
void SoftADC_setVoltageReference(float voltage);
void SoftADC_setThresholds(uint16_t chargeThresh, uint16_t dischargeThresh);
void SoftADC_setSteinHartCoefficients(float a, float b, float c);

// Utility functions
float SoftADC_readVoltage(uint8_t channel);
uint32_t SoftADC_getChargeCount(uint8_t channel);
uint32_t SoftADC_getDischargeCount(uint8_t channel);
bool SoftADC_isChannelActive(uint8_t channel);
bool SoftADC_isCharging(uint8_t channel);

// Error handling
typedef enum {
    SOFTADC_OK = 0,
    SOFTADC_ERROR_INVALID_CHANNEL,
    SOFTADC_ERROR_CHANNEL_INACTIVE,
    SOFTADC_ERROR_INVALID_PARAMETER,
    SOFTADC_ERROR_MEASUREMENT_TIMEOUT
} SoftADC_Error;

SoftADC_Error SoftADC_getLastError(void);
const char* SoftADC_getErrorString(SoftADC_Error error);

#ifdef __cplusplus
extern "C" {
#endif

// Any C-specific declarations would go here

#ifdef __cplusplus
}
#endif

#endif // SOFTADC_H
