#include <Arduino.h>
#include <cmath>

// Pin definitions
const int CHARGE_PIN = 5;    // Digital pin for charging
const int MEASURE_PIN = 34;  // Analog pin for voltage measurement

// Known parameters
const float CAPACITANCE = 1e-6;   // Capacitance in Farads
const float VCC = 3.3;            // Supply voltage in Volts
const float ADC_RESOLUTION = 4096.0; // 12-bit ADC resolution
const float QUANTIZATION_STEP = VCC / ADC_RESOLUTION; // ADC step size
const int NUM_SAMPLES = 10;       // Number of ADC samples for averaging

// Adaptive pulse width
float pulseWidthUs = 50;              // Initial pulse width in microseconds
const float MAX_PULSE_WIDTH_US = 500; // Maximum pulse width
const float MIN_PULSE_WIDTH_US = 10;  // Minimum pulse width

// Kalman filter parameters
float kalmanGain = 0.1; // Initial Kalman gain
float kalmanEstimate = 0;

// Last resistance estimate
float lastResistanceEstimate = 1e3; // 1 kOhm

// Get CPU cycle count
inline uint32_t getCycleCount() {
  uint32_t cycles;
  asm volatile("rsr %0,ccount" : "=a"(cycles));
  return cycles;
}

// Precise delay in CPU cycles
void delayCycles(uint32_t cycles) {
  uint32_t start = getCycleCount();
  while ((getCycleCount() - start) < cycles);
}

// Average ADC readings with Kalman filtering
float averageADC(int pin) {
  float sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    float raw = analogRead(pin) * (VCC / ADC_RESOLUTION);
    // Apply Kalman filter
    kalmanEstimate = kalmanEstimate + kalmanGain * (raw - kalmanEstimate);
    sum += kalmanEstimate;
    delayMicroseconds(50);
  }
  return sum / NUM_SAMPLES;
}

// Quantization-aware charging equation
float chargingODE(float vc, float R, float quantizationBias) {
  return (VCC - vc - quantizationBias) / (R * CAPACITANCE);
}

// Solve differential equation with quantization bias correction
float solveChargingCurve(float targetVoltage) {
  pinMode(CHARGE_PIN, OUTPUT);
  digitalWrite(CHARGE_PIN, LOW); // Ensure capacitor is discharged
  delay(10);                     // Allow discharge
  pinMode(CHARGE_PIN, INPUT);    // Tri-state mode

  float estimatedR = lastResistanceEstimate; // Start with last estimate
  float vc = 0;           // Initial capacitor voltage
  float t = 0;            // Time accumulator
  float dt = pulseWidthUs * 1e-6; // Initial step size
  int iterations = 0;

  // Accumulate voltage-time data
  const int maxDataPoints = 500;
  float timeData[maxDataPoints] = {0};
  float voltageData[maxDataPoints] = {0};
  int dataIndex = 0;

  while (vc < targetVoltage && iterations < 1000) {
    // Apply precise pulse
    uint32_t pulseCycles = (uint32_t)(pulseWidthUs * (float)ESP.getCpuFreqMHz());
    pinMode(CHARGE_PIN, OUTPUT);
    digitalWrite(CHARGE_PIN, HIGH);
    delayCycles(pulseCycles);
    digitalWrite(CHARGE_PIN, LOW);
    pinMode(CHARGE_PIN, INPUT);

    // Measure voltage
    float rawADC = averageADC(MEASURE_PIN);
    float measuredVoltage = rawADC * (VCC / ADC_RESOLUTION);

    // Store data
    if (dataIndex < maxDataPoints) {
      timeData[dataIndex] = t;
      voltageData[dataIndex] = measuredVoltage;
      dataIndex++;
    }

    // Compute quantization bias
    float quantizationBias = fmod(vc, QUANTIZATION_STEP) - QUANTIZATION_STEP / 2;

    // Solve charging ODE with quantization correction
    float theoreticalVoltage = vc + chargingODE(vc, estimatedR, quantizationBias) * dt;

    // Adjust resistance estimate
    float deltaV = measuredVoltage - vc;
    float theoreticalDeltaV = theoreticalVoltage - vc;
    if (fabs(deltaV - theoreticalDeltaV) > QUANTIZATION_STEP) {
      estimatedR *= deltaV / theoreticalDeltaV;
    }

    // Update variables
    t += dt;
    vc = measuredVoltage;
    pulseWidthUs = constrain(pulseWidthUs * (1 + 0.1 * (deltaV - theoreticalDeltaV)), MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);
    dt = pulseWidthUs * 1e-6;

    iterations++;
  }

  // Perform nonlinear least squares optimization to refine resistance
  float sumT = 0, sumT2 = 0, sumV = 0, sumTV = 0;
  for (int i = 0; i < dataIndex; i++) {
    sumT += timeData[i];
    sumT2 += timeData[i] * timeData[i];
    sumV += log(VCC - voltageData[i]);
    sumTV += timeData[i] * log(VCC - voltageData[i]);
  }
  float slope = (dataIndex * sumTV - sumT * sumV) / (dataIndex * sumT2 - sumT * sumT);
  estimatedR = -1 / (slope * CAPACITANCE);

  lastResistanceEstimate = estimatedR;
  return estimatedR;
}

void setup() {
  Serial.begin(115200);
  pinMode(CHARGE_PIN, OUTPUT);
  digitalWrite(CHARGE_PIN, LOW);
}

void loop() {
  float targetVoltage = VCC * 0.75; // 75% of VCC
  float resistance = solveChargingCurve(targetVoltage);

  Serial.print("Estimated Resistance: ");
  Serial.println(resistance, 5);

  delay(1000);
}
