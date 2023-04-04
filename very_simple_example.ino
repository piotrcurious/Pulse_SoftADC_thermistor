
// Simple example of using the SoftADC thermistor library with a fixed pulse length
// by Bing

// Include the SoftADC library
#include <Pulse_SoftADC_thermistor.h>

// Define the number of thermistors
#define NUM_THERMISTORS 2

// Define the pins for the thermistors
uint8_t pins[NUM_THERMISTORS] = {A0, A1};

// Define the pulse length in microseconds
#define PULSE_LENGTH 10

// Setup function
void setup() {
  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600);
  
  // Initialize the SoftADC library with the pulse length
  SoftADC_begin(pins, NUM_THERMISTORS, PULSE_LENGTH);
}

// Loop function
void loop() {
  // Update the SoftADC library and perform measurements
  SoftADC_update();
  
  // Declare a variable to store the last time the serial output was printed in milliseconds
  static unsigned long lastPrintTime;
  
  // Check if one second has passed since the last serial output
  if (millis() - lastPrintTime >= 1000) {
    // Update the last print time to the current time
    lastPrintTime = millis();
    
    // Print a new line and a separator
    Serial.println();
    Serial.println("----------");
    
    // Loop through all the thermistors
    for (uint8_t i = 0; i < NUM_THERMISTORS; i++) {
      // Calculate and print the resistance of the thermistor using SoftADC_calculateResistance function
      float resistance = SoftADC_calculateResistance(i);
      Serial.print("Resistance of thermistor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(resistance);
      Serial.println(" ohms");
    }
  }
}
