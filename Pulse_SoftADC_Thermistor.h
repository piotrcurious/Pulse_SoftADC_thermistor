
// SoftADC thermistor library for Arduino Uno
// by Bing

#ifndef SoftADC_h
#define SoftADC_h

#include <Arduino.h>

// Initialize the library with the given pins and pulse length
void SoftADC_begin(uint8_t pins[], uint8_t numPins, uint16_t pulseLen = DEFAULT_PULSE_LENGTH);

// Update the library and perform measurements
void SoftADC_update();

// Get the measurement of a channel in pulses
uint32_t SoftADC_read(uint8_t channel);

// Set a channel to inactive or active
void SoftADC_setActive(uint8_t channel, bool state);

// Calculate the resistance of the thermistor from the pulse average using a linear approximation formula
float SoftADC_calculateResistance(uint8_t channel);

#endif
