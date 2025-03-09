// SoftADC thermistor library for Arduino Uno
// Improved version with accurate RC calculations
// ESP32 version with analogReadMilliVolts and filtered analog reads

#include <Arduino.h>
#include <esp_adc_cal.h> // Required for ESP32 ADC calibration

#define MAX_CHANNELS 8
#define CHARGE_THRESHOLD_VOLTAGE 2.1f // ~4.4V threshold in Volts
#define DISCHARGE_THRESHOLD_VOLTAGE 0.8f // ~0.5V threshold in Volts
#define DEFAULT_PULSE_LENGTH 100
#define VCC 3.3f // Supply voltage in volts
//#define CAPACITOR_VALUE 10 * 0.000001f // 10uF capacitor value in Farads
#define CAPACITOR_VALUE 470 * 0.000001f // 10uF capacitor value in Farads

#define FILTER_WINDOW_SIZE 256 // Size of the moving average filter window

typedef struct {
    uint8_t pin;
    uint64_t chargeCount;
    uint64_t dischargeCount;
    uint64_t chargeAverage;
    uint64_t dischargeAverage;
    bool charging;
    bool active;
    float lastVoltage;          // Store last measured voltage
    unsigned long lastTime;      // Store last measurement time
    float chargeVoltage;         // Store charge voltage
    float dischargeVoltage;        // Store discharge voltage
    float voltageHistory[FILTER_WINDOW_SIZE]; // History for moving average filter
    uint8_t historyIndex;        // Index for history buffer
} Channel;

Channel channels[MAX_CHANNELS];
uint16_t pulseLength;

// Initialize the library
void SoftADC_begin(const uint8_t pins[], const uint8_t numPins, uint16_t pulseLen = DEFAULT_PULSE_LENGTH) {
    pulseLength = pulseLen;

    for (uint8_t i = 0; i < numPins; i++) {
        channels[i].pin = pins[i];
        pinMode(channels[i].pin, INPUT);
        digitalWrite(channels[i].pin, LOW);
        channels[i].chargeCount = 0;
        channels[i].dischargeCount = 0;
        channels[i].chargeAverage = 0;
        channels[i].dischargeAverage = 0;
        channels[i].charging = false;
        channels[i].active = true;
        channels[i].lastVoltage = 0;
        channels[i].lastTime = 0;
        channels[i].chargeVoltage = 0;
        channels[i].dischargeVoltage = 0;
        channels[i].historyIndex = 0;
        for (int j = 0; j < FILTER_WINDOW_SIZE; j++) {
            channels[i].voltageHistory[j] = 0.0f; // Initialize history buffer
        }
    }
}

// Filtered voltage reading function using moving average
float readVoltage(uint8_t pin, Channel* channelData) {
    float rawVoltage = analogReadMilliVolts(pin) / 1000.0f; // Read in mV and convert to Volts

    channelData->voltageHistory[channelData->historyIndex] = rawVoltage; // Store new reading
    channelData->historyIndex = (channelData->historyIndex + 1) % FILTER_WINDOW_SIZE; // Increment and wrap index

    float filteredVoltage = 0.0f;
    for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
        filteredVoltage += channelData->voltageHistory[i]; // Sum readings in history
    }
    filteredVoltage /= FILTER_WINDOW_SIZE; // Calculate average

    return filteredVoltage;
}

void SoftADC_update() {
    unsigned long currentTime = micros();
    static portMUX_TYPE _spinlock = portMUX_INITIALIZER_UNLOCKED;

    for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
        if (channels[i].active) {
            // Read current voltage with filtering
            float voltage ; 
            unsigned long deltaTime = currentTime - channels[i].lastTime;

            if (channels[i].charging) {
                taskENTER_CRITICAL(&_spinlock); // Disable FreeRTOS preemption
                pinMode(channels[i].pin, OUTPUT);
                digitalWrite(channels[i].pin, HIGH);
                channels[i].chargeCount++;

                unsigned long startTime = micros(); // Capture start time for non-blocking delay
                while ((micros() - startTime) < pulseLength) {
                    // Wait for pulseLength microseconds using micros()
                    // No delay function here, just spin-wait.
                }
                pinMode(channels[i].pin, INPUT);
                taskEXIT_CRITICAL(&_spinlock); // re-enable FreeRTOS preemption

                voltage = readVoltage(channels[i].pin, &channels[i]);

                if (voltage >= CHARGE_THRESHOLD_VOLTAGE) {
                    channels[i].charging = false;
                    channels[i].chargeAverage =
                        (channels[i].chargeAverage + channels[i].chargeCount) / 2;
                    channels[i].chargeCount = 0;
                    channels[i].chargeVoltage = voltage; // Store charge voltage here

                }
            } else {
                taskENTER_CRITICAL(&_spinlock); // Disable FreeRTOS preemption
   //             digitalWrite(channels[i].pin, LOW);
                pinMode(channels[i].pin, OUTPUT);
                digitalWrite(channels[i].pin, LOW);
                channels[i].dischargeCount++;

                unsigned long startTime = micros(); // Capture start time for non-blocking delay
                while ((micros() - startTime) < pulseLength) {
                    // Wait for pulseLength microseconds using micros()
                    // No delay function here, just spin-wait.
                }
                pinMode(channels[i].pin, INPUT);
                taskEXIT_CRITICAL(&_spinlock); // re-enable FreeRTOS preemption
                voltage = readVoltage(channels[i].pin, &channels[i]);

                if (voltage <= DISCHARGE_THRESHOLD_VOLTAGE) {
                    channels[i].charging = true;
                    channels[i].dischargeAverage =
                        (channels[i].dischargeAverage + channels[i].dischargeCount) / 2;
                    channels[i].dischargeCount = 0;
                    channels[i].dischargeVoltage = voltage; // Store discharge voltage here
                }
            }

            // Store current values for next iteration
            channels[i].lastVoltage = voltage;
            channels[i].lastTime = currentTime;
        }
    }
}
// Get raw measurement (same as before)
uint32_t SoftADC_read(uint8_t channel) {
    if (channel < MAX_CHANNELS && channels[channel].active) {
        return (channels[channel].chargeAverage + channels[channel].dischargeAverage) / 2;
    }
    return 0;
}

// Set channel active state (same as before)
void SoftADC_setActive(uint8_t channel, bool state) {
    if (channel < MAX_CHANNELS) {
        channels[channel].active = state;
        channels[channel].chargeCount = 0;
        channels[channel].dischargeCount = 0;
        channels[channel].chargeAverage = 0;
        channels[channel].dischargeAverage = 0;
        channels[channel].charging = false;
        channels[channel].lastVoltage = 0;
        channels[channel].lastTime = 0;
        channels[channel].chargeVoltage = 0;
        channels[channel].dischargeVoltage = 0;
        channels[channel].historyIndex = 0;
        for (int j = 0; j < FILTER_WINDOW_SIZE; j++) {
            channels[channel].voltageHistory[j] = 0.0f; // Initialize history buffer
        }
        pinMode(channels[channel].pin, INPUT);
        digitalWrite(channels[channel].pin, LOW);
    }
}


// Improved resistance calculation using RC circuit equations
float SoftADC_calculateResistance(uint8_t channel) {
    if (channel < MAX_CHANNELS && channels[channel].active) {
        uint32_t pulseCount = SoftADC_read(channel);

        float resistance = 0.0;
        float chargeResistance = 0.0;
        float dischargeResistance = 0.0;

        if (channels[channel].chargeAverage > 0 && channels[channel].dischargeAverage > 0 && channels[channel].chargeVoltage > 0 && channels[channel].dischargeVoltage > 0) {
            double pulseDurationCharge = (pulseLength * (double)channels[channel].chargeAverage) / 1000000.0;
            double pulseDurationDischarge = (pulseLength * (double)channels[channel].dischargeAverage) / 1000000.0;

            float chargeVoltage = channels[channel].chargeVoltage;
            float dischargeVoltage = channels[channel].dischargeVoltage;

            // Calculate charge resistance using the corrected formula
            if ((VCC - chargeVoltage) <= 0 || (VCC - dischargeVoltage) <= 0) {
                chargeResistance = 0.0; // Avoid invalid log argument
            } else {
                 chargeResistance = -pulseDurationCharge / (CAPACITOR_VALUE * log((VCC - chargeVoltage) / (VCC - dischargeVoltage)));
            }


            // Calculate discharge resistance using the corrected formula
            if (dischargeVoltage <= 0 || chargeVoltage <= 0) {
                dischargeResistance = 0.0; // Avoid invalid log argument
            } else {
                dischargeResistance = pulseDurationDischarge / (CAPACITOR_VALUE * log(chargeVoltage / dischargeVoltage));
            }


            resistance = (chargeResistance + dischargeResistance) / 2.0;
    //      resistance = chargeResistance; // Or use only charge or discharge, depending on which is more stable in your setup
            //resistance = dischargeResistance; // Or use only charge or discharge, depending on which is more stable in your setup


        } else {
            resistance = 0.0;
        }

        return abs(resistance); // Return absolute value to handle numerical instabilities
    }
    return 0.0;
}

// New function to get estimated temperature using Steinhart-Hart equation
float SoftADC_calculateTemperature(uint8_t channel) {
    float resistance = SoftADC_calculateResistance(channel);
    if (resistance > 0) {
        // Steinhart-Hart coefficients (these need to be calibrated for your specific thermistor)
        const float A = 0.001129148;
        const float B = 0.000234125;
        const float C = 0.0000000876741;
        
        // Steinhart-Hart equation: 1/T = A + B*ln(R) + C*(ln(R))^3
        float logR = log(resistance);
        float temperature = 1.0 / (A + B * logR + C * logR * logR * logR);
        
        // Convert from Kelvin to Celsius
        temperature = temperature - 273.15;
        
        return temperature;
    }
    return 0.0;
}
