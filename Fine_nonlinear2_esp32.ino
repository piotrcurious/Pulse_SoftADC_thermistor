
#include <driver/timer.h>

// Define the pins and constants
const int chargePin = 13; // The pin that charges the capacitor
const int measurePin = A0; // The pin that measures the capacitor voltage
const int ledPin = 12; // The pin that indicates the charging status
const float capValue = 0.0001; // The value of the capacitor in farads
const float threshold = 4.0; // The voltage threshold in volts
const uint32_t roughPulseCycles = 240000; // Rough pulse duration in CPU cycles (10ms at 24MHz)
const uint32_t finePulseCycles = 24000; // Fine pulse duration in CPU cycles (1ms at 24MHz)

// Declare the variables
uint32_t roughPulseCount = 0; // The number of rough pulses sent to the capacitor
uint32_t finePulseCount = 0; // The number of fine pulses sent to the capacitor
float capVoltage = 0.0; // The voltage across the capacitor
float resValue = 0.0; // The resistance value in ohms
bool fineMode = false; // The flag that indicates the fine mode

void setup() {
  // Initialize the pins
  pinMode(ledPin, OUTPUT);
  pinMode(measurePin, INPUT);
  digitalWrite(ledPin, LOW);
  
  // Initialize the serial monitor
  Serial.begin(115200);

  // Ensure CPU cycle counter is enabled
  esp_timer_init();
}

void delayCycles(uint32_t cycles) {
  uint32_t start = esp_timer_get_time() * 240; // Convert microseconds to CPU cycles
  while ((esp_timer_get_time() * 240 - start) < cycles) {
    // Busy wait
  }
}

void loop() {
  // Charge the capacitor with discrete pulses
  if (fineMode) {
    // Use a fine pulse to charge and discharge the capacitor
    pinMode(chargePin, OUTPUT); // Set the charge pin to output mode
    digitalWrite(chargePin, HIGH); // Send a high pulse
    delayCycles(finePulseCycles); // Wait for a precise number of cycles
    pinMode(chargePin, INPUT); // Set the charge pin to input mode
    delayMicroseconds(finePulseCycles / 240); // Corresponding delay
    finePulseCount++; // Increment the fine pulse count
    
    // Measure the capacitor voltage
    capVoltage = analogRead(measurePin) * (5.0 / 1023.0); // Convert the analog reading to volts
    
    // Check if the capacitor voltage is above or below the threshold
    if (capVoltage > threshold) {
      // Discharge the capacitor slightly by sending a low pulse
      pinMode(chargePin, OUTPUT); // Set the charge pin to output mode
      digitalWrite(chargePin, LOW);
      delayMicroseconds(finePulseCycles / 240);
      pinMode(chargePin, INPUT); // Set the charge pin to input mode
      capVoltage -= (5.0 / 1023.0); // Subtract a bit value from the voltage
    } else if (capVoltage < threshold) {
      // Charge the capacitor slightly by sending a high pulse
      pinMode(chargePin, OUTPUT); // Set the charge pin to output mode
      digitalWrite(chargePin, HIGH);
      delayMicroseconds(finePulseCycles / 240);
      pinMode(chargePin, INPUT); // Set the charge pin to input mode
      capVoltage += (5.0 / 1023.0); // Add a bit value to the voltage
    }
    
    // Check if the capacitor voltage is close to the threshold
    if (abs(capVoltage - threshold) < (5.0 / 1023.0)) {
      // Turn on the LED to indicate charging is done
      digitalWrite(ledPin, HIGH);
      
      // Calculate the total pulse time using cycle counts
      float totalPulseTime = (roughPulseCount * roughPulseCycles + finePulseCount * finePulseCycles * 2.0) / 240000000.0;
      
      // Calculate the resistance value using the formula: R = (t / C) / ln(Vc / V)
      resValue = (totalPulseTime / capValue) / log(capVoltage / 5.0);
      
      // Print the results to the serial monitor
      Serial.print("Rough pulse count: ");
      Serial.println(roughPulseCount);
      Serial.print("Fine pulse count: ");
      Serial.println(finePulseCount);
      Serial.print("Capacitor voltage: ");
      Serial.print(capVoltage);
      Serial.println(" V");
      Serial.print("Resistance value: ");
      Serial.print(resValue);
      Serial.println(" ohms");
      
      // Reset the variables and wait for a new measurement
      roughPulseCount = 0;
      finePulseCount = 0;
      capVoltage = 0.0;
      resValue = 0.0;
      fineMode = false;
      delayMicroseconds(5000000); // Wait for 5 seconds
      
      // Turn off the LED to indicate ready for a new measurement
      digitalWrite(ledPin, LOW);
    }
    
  } else {
    // Use a rough pulse to charge the capacitor quickly
    pinMode(chargePin, OUTPUT); // Set the charge pin to output mode
    digitalWrite(chargePin, HIGH); // Send a high pulse
    delayMicroseconds(roughPulseCycles / 240); // Wait for a longer time
    pinMode(chargePin, INPUT); // Set the charge pin to input mode
    roughPulseCount++; // Increment the rough pulse count
    
    // Measure the capacitor voltage
    capVoltage = analogRead(measurePin) * (5.0 / 1023.0); // Convert the analog reading to volts
    
    // Check if the capacitor voltage is above the threshold
    if (capVoltage >= threshold) {
      // Switch to the fine mode to refine the measurement
      fineMode = true;
    }
  }
}
