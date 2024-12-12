#include <Arduino.h>
#include <cmath>

// Pin definitions
const int CHARGE_PIN = 5;    // Digital pin for charging
const int MEASURE_PIN = 34;  // Analog pin for voltage measurement

// Known parameters
const double CAPACITANCE = 1e-6;   // Capacitance in Farads
const double VCC = 3.3;            // Supply voltage in Volts
const double ADC_RESOLUTION = 4096.0; // 12-bit ADC resolution
const double QUANTIZATION_STEP = VCC / ADC_RESOLUTION; // ADC step size
const int NUM_SAMPLES = 10;       // Number of ADC samples for averaging

// Adaptive pulse width
double pulseWidthUs = 50;              // Initial pulse width in microseconds
const double MAX_PULSE_WIDTH_US = 500; // Maximum pulse width
const double MIN_PULSE_WIDTH_US = 10;  // Minimum pulse width

// Kalman filter parameters
double kalmanGain = 0.1; // Initial Kalman gain
double kalmanEstimate = 0;
double kalmanVariance = 0.01; // Initial noise variance

// Last resistance estimate
double lastResistanceEstimate = 1e3; // 1 kOhm

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

// Adaptive Kalman filter for ADC readings
double adaptiveKalmanFilter(double rawADC) {
  static double previousVariance = kalmanVariance;

  // Compute error covariance and Kalman gain
  double predictedVariance = previousVariance + 0.01; // Predicted error growth
  kalmanGain = predictedVariance / (predictedVariance + QUANTIZATION_STEP);
  
  // Update estimate and variance
  kalmanEstimate += kalmanGain * (rawADC - kalmanEstimate);
  kalmanVariance = (1 - kalmanGain) * predictedVariance;

  previousVariance = kalmanVariance;
  return kalmanEstimate;
}

// Average ADC readings with Kalman filtering
double averageADC(int pin) {
  double sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    double raw = analogRead(pin) * (VCC / ADC_RESOLUTION);
    sum += adaptiveKalmanFilter(raw);
    delayMicroseconds(50);
  }
  return sum / NUM_SAMPLES;
}

// Adaptive Runge-Kutta-Fehlberg method for solving ODE
double solveRKF(double vc, double R, double dt, double quantizationBias) {
  auto f = [&](double v) {
    return (VCC - v - quantizationBias) / (R * CAPACITANCE);
  };

  double k1 = f(vc);
  double k2 = f(vc + 0.5 * dt * k1);
  double k3 = f(vc + 0.5 * dt * k2);
  double k4 = f(vc + dt * k3);

  return vc + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
}

// Solve charging curve iteratively with adaptive pulse width
double solveChargingCurve(double targetVoltage) {
  pinMode(CHARGE_PIN, OUTPUT);
  digitalWrite(CHARGE_PIN, LOW); // Ensure capacitor is discharged
  delay(10);                     // Allow discharge
  pinMode(CHARGE_PIN, INPUT);    // Tri-state mode

  double estimatedR = lastResistanceEstimate; // Start with last estimate
  double vc = 0;           // Initial capacitor voltage
  double t = 0;            // Time accumulator
  double dt = pulseWidthUs * 1e-6; // Initial step size
  int iterations = 0;

  // Accumulate voltage-time data
  const int maxDataPoints = 500;
  double timeData[maxDataPoints] = {0};
  double voltageData[maxDataPoints] = {0};
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
    double rawADC = averageADC(MEASURE_PIN);
    double measuredVoltage = rawADC * (VCC / ADC_RESOLUTION);

    // Store data
    if (dataIndex < maxDataPoints) {
      timeData[dataIndex] = t;
      voltageData[dataIndex] = measuredVoltage;
      dataIndex++;
    }

    // Compute quantization bias
    double quantizationBias = fmod(vc, QUANTIZATION_STEP) - QUANTIZATION_STEP / 2;

    // Solve ODE with adaptive RKF
    double theoreticalVoltage = solveRKF(vc, estimatedR, dt, quantizationBias);

    // Adjust resistance estimate
    double deltaV = measuredVoltage - vc;
    double theoreticalDeltaV = theoreticalVoltage - vc;
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

  // Nonlinear optimization for resistance estimate
  double sumT = 0, sumT2 = 0, sumV = 0, sumTV = 0;
  for (int i = 0; i < dataIndex; i++) {
    sumT += timeData[i];
    sumT2 += timeData[i] * timeData[i];
    sumV += log(VCC - voltageData[i]);
    sumTV += timeData[i] * log(VCC - voltageData[i]);
  }
  double slope = (dataIndex * sumTV - sumT * sumV) / (dataIndex * sumT2 - sumT * sumT);
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
  double targetVoltage = VCC * 0.75; // 75% of VCC
  double resistance = solveChargingCurve(targetVoltage);

  Serial.print("Estimated Resistance: ");
  Serial.println(resistance, 8);

  delay(1000);
}
