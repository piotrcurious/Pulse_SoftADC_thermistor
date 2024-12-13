
#include <driver/timer.h>

// Define the pins and constants
const int chargePin = 13; // The pin that charges the capacitor
const int measurePin = A0; // The pin that measures the capacitor voltage
const int ledPin = 12; // The pin that indicates the charging status
const float capValue = 0.0001; // The value of the capacitor in farads
const float threshold = 4.0; // The voltage threshold in volts
const uint32_t roughPulseCycles = 240000; // Rough pulse duration in CPU cycles (10ms at 24MHz)
const uint32_t finePulseCycles = 2400; // Fine pulse duration in CPU cycles (0.1ms at 24MHz)
const float ADC_RESOLUTION = 5.0 / 1023.0; // ADC voltage per bit
const int MAX_FINE_STEPS = 100; // Maximum number of fine adjustment steps

// Declare the variables
uint32_t roughPulseCount = 0; // The number of rough pulses sent to the capacitor
uint32_t finePulseCount = 0; // The number of fine pulses sent to the capacitor
float capVoltage = 0.0; // The voltage across the capacitor
float resValue = 0.0; // The resistance value in ohms
bool fineMode = false; // The flag that indicates the fine mode
int fineStepDirection = 0; // Direction and magnitude of fine steps

void setup() {
  // Initialize the pins
  pinMode(ledPin, OUTPUT);
  pinMode(measurePin, INPUT);
  pinMode(chargePin, OUTPUT);
  digitalWrite(ledPin, LOW);
  digitalWrite(chargePin, LOW);
  
  // Initialize the serial monitor
  Serial.begin(115200);

  // Ensure CPU cycle counter is enabled
  esp_timer_init();
}

uint32_t delayCycles(uint32_t cycles) {
  uint32_t start = esp_timer_get_time() * 240; // Convert microseconds to CPU cycles
  uint32_t end = start + cycles;
  while ((esp_timer_get_time() * 240) < end) {
    // Busy wait
  }
  return cycles;
}

float measureCapacitorVoltage() {
  // Take multiple readings and average them for better precision
  float voltageSum = 0;
  const int numReadings = 10;
  
  for (int i = 0; i < numReadings; i++) {
    voltageSum += analogRead(measurePin) * ADC_RESOLUTION;
    delayMicroseconds(10); // Small delay between readings
  }
  
  return voltageSum / numReadings;
}

void adjustCapacitorVoltage() {
  // More sophisticated voltage adjustment
  float currentVoltage = measureCapacitorVoltage();
  int stepsTaken = 0;
  
  // Determine the direction and magnitude of adjustment
  if (currentVoltage > threshold) {
    fineStepDirection = -1; // Discharge
  } else {
    fineStepDirection = 1; // Charge
  }
  
  // Iterative fine adjustment
  while (abs(currentVoltage - threshold) > ADC_RESOLUTION && stepsTaken < MAX_FINE_STEPS) {
    // Alternate between high and low pulses for more precise control
    pinMode(chargePin, OUTPUT);
    if (fineStepDirection > 0) {
      digitalWrite(chargePin, HIGH);
    } else {
      digitalWrite(chargePin, LOW);
    }
    
    // Very short pulse
    delayCycles(finePulseCycles);
    pinMode(chargePin, INPUT);
    
    // Measure new voltage
    float newVoltage = measureCapacitorVoltage();
    
    // Update tracking
    currentVoltage = newVoltage;
    stepsTaken++;
    finePulseCount++;
  }
  
  return;
}

void loop() {
  // Charge the capacitor with discrete pulses
  if (!fineMode) {
    // Use a rough pulse to charge the capacitor quickly
    digitalWrite(chargePin, HIGH);
    delayCycles(roughPulseCycles);
    digitalWrite(chargePin, LOW);
    roughPulseCount++;
    
    // Measure the capacitor voltage
    capVoltage = measureCapacitorVoltage();
    
    // Check if the capacitor voltage is near the threshold
    if (abs(capVoltage - threshold) < (2 * ADC_RESOLUTION)) {
      fineMode = true;
    }
  } else {
    // Fine-tuning mode
    adjustCapacitorVoltage();
    
    // Check if we've reached a stable state
    capVoltage = measureCapacitorVoltage();
    
    // Determine if we've reached a stable voltage near the threshold
    if (abs(capVoltage - threshold) < ADC_RESOLUTION) {
      // Turn on the LED to indicate charging is done
      digitalWrite(ledPin, HIGH);
      
      // Calculate the total pulse time using cycle counts
      float totalPulseTime = (roughPulseCount * roughPulseCycles + finePulseCount * finePulseCycles) / 240000000.0;
      
      // Calculate the resistance value using the formula: R = (t / C) / ln(Vc / V)
      resValue = (totalPulseTime / capValue) / log(capVoltage / 5.0);
      
      // Print detailed results to the serial monitor
      Serial.println("Measurement Complete:");
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
      
      // Reset for next measurement
      roughPulseCount = 0;
      finePulseCount = 0;
      fineMode = false;
      fineStepDirection = 0;
      
      // Wait before next measurement
      delayMicroseconds(5000000); // 5 seconds
      
      // Turn off the LED
      digitalWrite(ledPin, LOW);
    }
  }
} 
