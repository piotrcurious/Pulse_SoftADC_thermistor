#include <Arduino.h>

// Pin definitions
const int CHARGE_PIN = 5;    // Digital pin to pulse for charging
const int MEASURE_PIN = 34;  // Analog pin to measure capacitor voltage

// Known parameters
const float CAPACITANCE = 1e-6; // Known capacitance in Farads
const float VCC = 3.3;          // Supply voltage in Volts
const int NUM_SAMPLES = 10;     // Number of samples for averaging
const float ADC_RESOLUTION = 4096.0; // 12-bit ADC resolution
const float TOLERANCE = 1e-5;   // Tolerance for voltage convergence

// Incremental charging parameters
const float PULSE_WIDTH_US = 50;  // Width of each charge pulse
const int MAX_ITERATIONS = 1000; // Maximum iterations to avoid infinite loops

// Function to calculate theoretical voltage increment
float calculateVoltageIncrement(float vc, float R, float dt) {
  return (VCC - vc) / (R * CAPACITANCE) * dt;
}

// Average multiple ADC readings for noise reduction
float averageADC(int pin) {
  float sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(50);
  }
  return sum / NUM_SAMPLES;
}

// Incremental charging process to refine resistance estimate
float incrementalCharge(float targetVoltage) {
  pinMode(CHARGE_PIN, OUTPUT);
  digitalWrite(CHARGE_PIN, LOW); // Ensure capacitor is initially discharged
  delay(10);                     // Allow full discharge
  pinMode(CHARGE_PIN, INPUT);    // Tri-state mode to minimize leakage

  float estimatedR = 1e3; // Initial guess for resistance in ohms
  float vc = 0;           // Initial capacitor voltage
  float dt = PULSE_WIDTH_US * 1e-6; // Convert pulse width to seconds
  int iterations = 0;

  while (vc < targetVoltage && iterations < MAX_ITERATIONS) {
    // Apply a single pulse to charge the capacitor incrementally
    pinMode(CHARGE_PIN, OUTPUT);
    digitalWrite(CHARGE_PIN, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(CHARGE_PIN, LOW);
    pinMode(CHARGE_PIN, INPUT);

    // Measure capacitor voltage
    float rawADC = averageADC(MEASURE_PIN);
    float measuredVoltage = rawADC * (VCC / ADC_RESOLUTION);

    // Refine resistance estimate using differential equation
    float dVc = measuredVoltage - vc;
    float theoreticalIncrement = calculateVoltageIncrement(vc, estimatedR, dt);

    if (fabs(dVc - theoreticalIncrement) > TOLERANCE) {
      estimatedR *= dVc / theoreticalIncrement; // Adjust resistance estimate
    }

    vc = measuredVoltage; // Update the voltage for the next step
    iterations++;
  }

  return estimatedR;
}

void setup() {
  Serial.begin(115200);
  pinMode(CHARGE_PIN, OUTPUT);
  digitalWrite(CHARGE_PIN, LOW);
}

void loop() {
  // Target voltage: 50% of VCC for mid-range measurements
  float targetVoltage = VCC / 2;

  // Measure and estimate resistance
  float estimatedResistance = incrementalCharge(targetVoltage);

  Serial.print("Estimated Resistance: ");
  Serial.println(estimatedResistance, 2);

  delay(1000); // Wait before the next cycle
}
