
// Example of using the SoftADC thermistor library to determine measurement noise level and apply kalman filters
// by Bing

// Include the SoftADC library
#include <Pulse_SoftADC_thermistor.h>

// Define the number of thermistors
#define NUM_THERMISTORS 2

// Define the pins for the thermistors
uint8_t pins[NUM_THERMISTORS] = {A0, A1};

// Define the minimum and maximum pulse lengths in microseconds
#define MIN_PULSE_LENGTH 1
#define MAX_PULSE_LENGTH 20

// Define the process noise level (change it according to your thermistor and capacitor values)
#define PROCESS_NOISE 0.1

// Define a struct to store the kalman filter data
typedef struct {
  float estimate; // the estimated value
  float error; // the estimation error
  float measurementNoise; // the measurement noise level
  float gain; // the kalman gain
} KalmanFilter;

// Declare an array of kalman filters for each thermistor
KalmanFilter filters[NUM_THERMISTORS];

// Declare an array of global averages for each thermistor
float globalAverages[NUM_THERMISTORS];

// Declare an array to store the measurements of each thermistor
float measurements[NUM_THERMISTORS];

// Declare an array to store the previous measurements of each thermistor
float prevMeasurements[NUM_THERMISTORS];

// Initialize the kalman filters with initial values
void initKalmanFilters() {
  // Loop through all the filters
  for (uint8_t i = 0; i < NUM_THERMISTORS; i++) {
    // Set the initial estimate to zero
    filters[i].estimate = 0.0;
    
    // Set the initial error to a large value
    filters[i].error = 1000.0;
    
    // Set the initial measurement noise level to zero
    filters[i].measurementNoise = 0.0;
    
    // Set the initial gain to zero
    filters[i].gain = 0.0;
    
    // Set the initial global average to zero
    globalAverages[i] = 0.0;
    
    // Set the initial measurement to zero
    measurements[i] = 0.0;
    
    // Set the initial previous measurement to zero
    prevMeasurements[i] = 0.0;
  }
}

// Update the kalman filters with a new measurement
void updateKalmanFilters() {
  // Loop through all the filters and measurements
  for (uint8_t i = 0; i < NUM_THERMISTORS; i++) {
    // Get the measurement of the thermistor resistance using SoftADC_calculateResistance function
    measurements[i] = SoftADC_calculateResistance(i);
    
    // Update the estimation error by adding the process noise level
    filters[i].error += PROCESS_NOISE;
    
    // Calculate the kalman gain as the ratio of estimation error and measurement noise
    filters[i].gain = filters[i].error / (filters[i].error + filters[i].measurementNoise);
    
    // Update the estimate by adding the product of kalman gain and measurement deviation
    filters[i].estimate += filters[i].gain * (measurements[i] - filters[i].estimate);
    
    // Update the estimation error by multiplying it with one minus kalman gain
    filters[i].error *= (1 - filters[i].gain);
    
    // Update the global average by adding the estimate and dividing by two
    globalAverages[i] = (globalAverages[i] + filters[i].estimate) / 2;
  }
}

// Change the pulse length randomly every five minutes and update the measurement noise level for each thermistor by comparing their resistances at different pulse lengths
void changePulseLength() {
  // Declare a variable to store the current pulse length
  static uint16_t currentPulseLength;
  
  // Declare a variable to store the last time the pulse length was changed in milliseconds
  static unsigned long lastChangeTime;
  
  // Check if this is the first time the function is called
  if (lastChangeTime == 0) {
    // Initialize the current pulse length to a random value between minimum and maximum
    currentPulseLength = random(MIN_PULSE_LENGTH, MAX_PULSE_LENGTH + 1);
    
    // Initialize the last change time to the current time
    lastChangeTime = millis();
    
    // Initialize the SoftADC library with the current pulse length
    SoftADC_begin(pins, NUM_THERMISTORS, currentPulseLength);
  }
  
  // Check if five minutes have passed since the last pulse length change
  if (millis() - lastChangeTime >= 5 * 60 * 1000) {
    // Change the current pulse length to a random value between minimum and maximum
    currentPulseLength = random(MIN_PULSE_LENGTH, MAX_PULSE_LENGTH + 1);
    
    // Update the last change time to the current time
    lastChangeTime = millis();
    
    // Initialize the SoftADC library with the current pulse length
    SoftADC_begin(pins, NUM_THERMISTORS, currentPulseLength);
    
    // Loop through all the thermistors
    for (uint8_t i = 0; i < NUM_THERMISTORS; i++) {
      // Calculate and print the difference between the current and previous measurement
      float difference = abs(measurements[i] - prevMeasurements[i]);
      Serial.print("Difference from previous measurement: ");
      Serial.print(difference);
      Serial.println(" ohms");
      
      // Update the measurement noise level of the kalman filter using a simple moving average formula
      filters[i].measurementNoise = (filters[i].measurementNoise + difference) / 2;
      Serial.print("Measurement noise level: ");
      Serial.println(filters[i].measurementNoise);
      
      // Store the current measurement as the previous measurement for the next iteration
      prevMeasurements[i] = measurements[i];
    }
  }
}

// Setup function
void setup() {
  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600);
  
  // Initialize the kalman filters with initial values
  initKalmanFilters();
}

// Loop function
void loop() {
  // Change the pulse length randomly every five minutes and update the measurement noise level for each thermistor by comparing their resistances at different pulse lengths
  changePulseLength();
  
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
  
  // Update the kalman filters with a new measurement
  updateKalmanFilters();
  
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
      // Print the measurement, estimate and global average of the thermistor resistance
      Serial.print("Measurement of thermistor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(measurements[i]);
      Serial.println(" ohms");
      
      Serial.print("Estimate of thermistor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(filters[i].estimate);
      Serial.println(" ohms");
      
      Serial.print("Global average of thermistor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(globalAverages[i]);
      Serial.println(" ohms");
    }
  }
}
