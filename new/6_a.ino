#include <Arduino.h>

// Pin definitions
const int CHARGE_PIN = 5;    // Digital pin to pulse for charging
const int MEASURE_PIN = 34;  // Analog pin to measure capacitor voltage

// Known parameters
const float CAPACITANCE = 1e-6;   // Known capacitance in Farads
const float VCC = 3.3;            // Supply voltage in Volts
const int NUM_SAMPLES = 20;       // Number of samples for noise reduction
const float ADC_RESOLUTION = 4096.0; // 12-bit ADC resolution
const float TOLERANCE = 1e-6;     // Tolerance for numerical integration
const float QUANTIZATION_STEP = VCC / ADC_RESOLUTION; // ADC step size

// Adaptive parameters
const float MAX_PULSE_WIDTH_US = 500; // Maximum pulse width in microseconds
const float MIN_PULSE_WIDTH_US = 5;   // Minimum pulse width in microseconds
float pulseWidthUs = 50;              // Initial pulse width in microseconds

// Last computed resistance (initialized to a reasonable guess)
float lastResistanceEstimate = 1e3; // 1 kOhm

// Get current CPU cycle count
inline uint32_t getCycleCount() {
  uint32_t cycles;
  asm volatile("rsr %0,ccount" : "=a"(cycles));
  return cycles;
}

// Wait for a precise number of CPU cycles
void delayCycles(uint32_t cycles) {
  uint32_t startCycle = getCycleCount();
  while ((getCycleCount() - startCycle) < cycles);
}

// Average ADC readings for noise reduction
float averageADC(int pin) {
  float sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(50);
  }
  return sum / NUM_SAMPLES;
}

// Differential equation for capacitor charging
float chargingODE(float vc, float R) {
  return (VCC - vc) / (R * CAPACITANCE);
}

// Runge-Kutta-Fehlberg method for numerical integration
float RKF45(float vc, float R, float dt) {
  float k1 = dt * chargingODE(vc, R);
  float k2 = dt * chargingODE(vc + 0.5 * k1, R);
  float k3 = dt * chargingODE(vc + 0.5 * k2, R);
  float k4 = dt * chargingODE(vc + k3, R);
  return vc + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
}

// Sub-quantization level estimation for ADC values
float refineADC(float rawADC) {
  float coarseVoltage = rawADC * (VCC / ADC_RESOLUTION);
  float fineAdjustment = QUANTIZATION_STEP / 2 * (random(-50, 50) / 100.0); // Simulate sub-quantization levels
  return coarseVoltage + fineAdjustment;
}

// Adaptive pulse length adjustment based on quantization errors
float adjustPulseWidth(float measuredDeltaV, float expectedDeltaV) {
  float error = fabs(measuredDeltaV - expectedDeltaV);
  if (error < QUANTIZATION_STEP / 2) {
    return min(pulseWidthUs * 1.1, MAX_PULSE_WIDTH_US); // Increase pulse width
  } else {
    return max(pulseWidthUs * 0.9, MIN_PULSE_WIDTH_US); // Decrease pulse width
  }
}

// Incremental charging with adaptive numerical methods
float accumulateChargingCurve(float targetVoltage) {
  pinMode(CHARGE_PIN, OUTPUT);
  digitalWrite(CHARGE_PIN, LOW); // Ensure capacitor is initially discharged
  delay(10);                     // Allow full discharge
  pinMode(CHARGE_PIN, INPUT);    // Tri-state mode to minimize leakage

  float estimatedR = lastResistanceEstimate; // Start with last computed estimate
  float vc = 0;           // Initial capacitor voltage
  float t = 0;            // Time accumulator
  int iterations = 0;

  while (vc < targetVoltage && iterations < 1000) {
    // Calculate pulse duration in cycles
    uint32_t pulseCycles = (uint32_t)(pulseWidthUs * (float)ESP.getCpuFreqMHz());

    // Apply precise pulse using cycle counter
    pinMode(CHARGE_PIN, OUTPUT);
    digitalWrite(CHARGE_PIN, HIGH);
    delayCycles(pulseCycles);
    digitalWrite(CHARGE_PIN, LOW);
    pinMode(CHARGE_PIN, INPUT);

    // Measure capacitor voltage with refined ADC reading
    float rawADC = averageADC(MEASURE_PIN);
    float measuredVoltage = refineADC(rawADC);

    // Compute theoretical voltage increment using RKF45
    float dt = pulseWidthUs * 1e-6;
    float theoreticalDeltaV = RKF45(vc, estimatedR, dt) - vc;
    float measuredDeltaV = measuredVoltage - vc;

    // Update resistance estimate using refined measurements
    if (fabs(measuredDeltaV - theoreticalDeltaV) > TOLERANCE) {
      estimatedR *= measuredDeltaV / theoreticalDeltaV; // Adjust resistance estimate
    }

    // Accumulate time and voltage
    t += dt;
    vc = measuredVoltage;

    // Adjust pulse width for next iteration
    pulseWidthUs = adjustPulseWidth(measuredDeltaV, theoreticalDeltaV);

    iterations++;
  }

  // Store the last computed resistance estimate for reuse
  lastResistanceEstimate = estimatedR;

  return estimatedR;
}

void setup() {
  Serial.begin(115200);
  pinMode(CHARGE_PIN, OUTPUT);
  digitalWrite(CHARGE_PIN, LOW);
}

void loop() {
  // Target voltage: 75% of VCC for a broader range
  float targetVoltage = VCC * 0.75;

  // Measure and estimate resistance
  float estimatedResistance = accumulateChargingCurve(targetVoltage);

  Serial.print("Estimated Resistance: ");
  Serial.println(estimatedResistance, 6); // Print with high precision

  delay(1000); // Wait before the next cycle
}
