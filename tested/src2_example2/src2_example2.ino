// SoftADC Library Example
// For Arduino Serial Plotter visualization
// Plots temperature and resistance measurements from a thermistor

#include "Pulse_ADC.h"

// Pin configuration
const int NUM_THERMISTORS = 1;  // Number of thermistors to read
const uint8_t THERMISTOR_PINS[NUM_THERMISTORS] = {32};  // Analog pins for thermistors

// Timing configuration
const unsigned long SAMPLE_INTERVAL = 200;  // Measurement interval in milliseconds
unsigned long lastSampleTime = 0;

void setup() {
  // Initialize serial communication at 115200 baud
  // Higher baud rate for smoother plotting
  Serial.begin(115200);
  
  // Initialize the SoftADC library
  SoftADC_begin(THERMISTOR_PINS, NUM_THERMISTORS,100);
  
  // Optional: Configure the library with your specific component values
  // SoftADC_setCapacitorValue(1e-7);  // 100nF capacitor
  // SoftADC_setVoltageReference(5.0);  // 5V reference
  
  // Print column headers for serial plotter
  // Using comma-separated format with labels
  Serial.println("Temperature(C),Resistance(ohm)");
}

void loop() {
  // Update the SoftADC measurements
  SoftADC_update();
  
  // Check if it's time for a new sample
  if (millis() - lastSampleTime >= SAMPLE_INTERVAL) {
    // Read temperature and resistance from the first thermistor
    float temperature = SoftADC_calculateTemperature(0);
    float resistance = SoftADC_calculateResistance(0);
    
    // Format data for serial plotter
    // The plotter expects comma-separated values on a single line
    Serial.print(temperature);
    Serial.print(",");
    Serial.println(resistance);
    
    // Update sample timing
    lastSampleTime = millis();
  }
  yield();
  // Optional: Add a small delay to prevent overwhelming the serial buffer
//  delay(random(20));
}

// Optional: Error checking function
void checkForErrors() {
  SoftADC_Error error = SoftADC_getLastError();
  if (error != SOFTADC_OK) {
    Serial.print("ERROR:");
    Serial.println(SoftADC_getErrorString(error));
  }
}
