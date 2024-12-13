
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

// Declare an array to store the resistances of each thermistor
float resistances[NUM_THERMISTORS];

// Setup function
void setup() {
  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600);
  
  // Initialize the SoftADC library with the pulse length
  SoftADC_begin(pins, NUM_THERMISTORS, PULSE_LENGTH);
}

// Loop function
void loop() {
  // Update the SoftADC library and perform measurements until at least one charge and discharge cycle is completed for each channel
  bool cycleCompleted = false;
  while (!cycleCompleted) {
    // Update the SoftADC library
    SoftADC_update();
    
    // Assume that the cycle is completed unless proven otherwise
    cycleCompleted = true;
    
    // Loop through all the channels and check their charging states
    for (uint8_t i = 0; i < NUM_THERMISTORS; i++) {
      // If any channel is still charging, set the cycleCompleted flag to false
      if (channels[i].charging) {
        cycleCompleted = false;
        break;
      }
    }
  }
  
  // Loop through all the thermistors
  for (uint8_t i = 0; i < NUM_THERMISTORS; i++) {
    // Calculate and store the resistance of the thermistor using SoftADC_calculateResistance function
    resistances[i] = SoftADC_calculateResistance(i);
  }
  
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
      // Print the resistance of the thermistor 
      Serial.print("Resistance of thermistor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(resistances[i]);
      Serial.println(" ohms");
    }
  }
}
