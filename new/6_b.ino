#include <Arduino.h>

// Pin definitions
const int CHARGE_PIN = 5;    // Digital pin to pulse for charging
const int MEASURE_PIN = 34;  // Analog pin to measure capacitor voltage

// Known parameters
const float CAPACITANCE = 1e-6;   // Capacitance in Farads
const float VCC = 3.3;            // Supply voltage in Volts
const int NUM_SAMPLES = 10;       // Samples for ADC averaging
const float ADC_RESOLUTION = 4096.0; // 12-bit ADC resolution
const float TOLERANCE = 1e-6;     // Convergence tolerance
const float QUANTIZATION_STEP = VCC / ADC_RESOLUTION; // ADC step size

// Adaptive parameters
const float MAX_PULSE_WIDTH_US = 500; // Maximum pulse width in microseconds
const float MIN_PULSE_WIDTH_US = 10;  // Minimum pulse width in microseconds
float pulseWidthUs = 50;              // Initial pulse width

// Last computed resistance estimate
float lastResistanceEstimate = 1e3; // 1 kOhm

// Get CPU cycle count
inline uint32_t getCycleCount() {
  uint32_t cycles;
  asm volatile("rsr %0,ccount" : "=a"(cycles));
  return cycles;
}

// Wait for a precise number of cycles
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

// Runge-Kutta-Fehlberg (RKF45) for numerical integration
float rkf45(float vc, float R, float dt) {
  float k1 = (VCC - vc) / (R * CAPACITANCE);
  float k2 = (VCC - (vc + 0.5 * dt * k1)) / (R * CAPACITANCE);
  float k3 = (VCC - (vc + 0.5 * dt * k2)) / (R * CAPACITANCE);
  float k4 = (VCC - (vc + dt * k3)) / (R * CAPACITANCE);
  return vc + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
}

// Accumulate charging curve and fit exponential model
float accumulateChargingCurve(float targetVoltage) {
  pinMode(CHARGE_PIN, OUTPUT);
  digitalWrite(CHARGE_PIN, LOW); // Ensure capacitor is discharged
  delay(10);                     // Allow full discharge
  pinMode(CHARGE_PIN, INPUT);    // Tri-state mode

  float estimatedR = lastResistanceEstimate; // Use last estimate
  float vc = 0;           // Initial capacitor voltage
  float t = 0;            // Time accumulator
  float dt = pulseWidthUs * 1e-6; // Initial time step
  int iterations = 0;

  // Arrays to store voltage-time data for curve fitting
  const int maxDataPoints = 500;
  float timeData[maxDataPoints] = {0};
  float voltageData[maxDataPoints] = {0};
  int dataIndex = 0;

  while (vc < targetVoltage && iterations < 1000) {
    // Apply adaptive pulse to charge incrementally
    uint32_t pulseCycles = (uint32_t)(pulseWidthUs * (float)ESP.getCpuFreqMHz());
    pinMode(CHARGE_PIN, OUTPUT);
    digitalWrite(CHARGE_PIN, HIGH);
    delayCycles(pulseCycles);
    digitalWrite(CHARGE_PIN, LOW);
    pinMode(CHARGE_PIN, INPUT);

    // Measure capacitor voltage
    float rawADC = averageADC(MEASURE_PIN);
    float measuredVoltage = rawADC * (VCC / ADC_RESOLUTION);

    // Store data for curve fitting
    if (dataIndex < maxDataPoints) {
      timeData[dataIndex] = t;
      voltageData[dataIndex] = measuredVoltage;
      dataIndex++;
    }

    // Integrate using RKF45
    float theoreticalVoltage = rkf45(vc, estimatedR, dt);

    // Adjust resistance estimate
    float deltaV = measuredVoltage - vc;
    float theoreticalDeltaV = theoreticalVoltage - vc;
    if (fabs(deltaV - theoreticalDeltaV) > TOLERANCE) {
      estimatedR *= deltaV / theoreticalDeltaV;
    }

    // Update time, voltage, and step size
    t += dt;
    vc = measuredVoltage;
    pulseWidthUs = constrain(pulseWidthUs * (1 + 0.1 * (deltaV - theoreticalDeltaV)), MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);
    dt = pulseWidthUs * 1e-6; // Adjust step size dynamically

    iterations++;
  }

  // Fit exponential curve to collected data (least squares)
  float sumT = 0, sumT2 = 0, sumV = 0, sumTV = 0;
  for (int i = 0; i < dataIndex; i++) {
    sumT += timeData[i];
    sumT2 += timeData[i] * timeData[i];
    sumV += log(VCC - voltageData[i]); // Linearize exponential
    sumTV += timeData[i] * log(VCC - voltageData[i]);
  }
  float slope = (dataIndex * sumTV - sumT * sumV) / (dataIndex * sumT2 - sumT * sumT);
  estimatedR = -1 / (slope * CAPACITANCE); // Estimate resistance

  // Store the last computed resistance estimate
  lastResistanceEstimate = estimatedR;

  return estimatedR;
}

void setup() {
  Serial.begin(115200);
  pinMode(CHARGE_PIN, OUTPUT);
  digitalWrite(CHARGE_PIN, LOW);
}

void loop() {
  // Target voltage: 75% of VCC
  float targetVoltage = VCC * 0.75;

  // Measure and estimate resistance
  float estimatedResistance = accumulateChargingCurve(targetVoltage);

  Serial.print("Estimated Resistance: ");
  Serial.println(estimatedResistance, 2);

  delay(1000); // Wait before the next cycle
}
