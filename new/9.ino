#include <Arduino.h>
#include <cmath>
#include <vector>

// Pin definitions
const int CHARGE_PIN = 5;    // Digital pin for charging
const int MEASURE_PIN = 34;  // Analog pin for voltage measurement

// Known parameters
const double CAPACITANCE = 1e-6;  // Capacitance in Farads
const double VCC = 3.3;           // Supply voltage in Volts
const double ADC_RESOLUTION = 4096.0; // 12-bit ADC resolution
const double QUANTIZATION_STEP = VCC / ADC_RESOLUTION; // ADC step size

// Adaptive pulse width
double pulseWidthUs = 50;           // Initial pulse width in microseconds
const double MAX_PULSE_WIDTH_US = 500; // Maximum pulse width
const double MIN_PULSE_WIDTH_US = 10;  // Minimum pulse width

// Kalman filter parameters
double kalmanEstimate = 0;
double kalmanVariance = 0.01;  // Initial noise variance

// CPU cycle counter
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
  static double predictedVariance = 0.01;

  // Compute Kalman gain
  double kalmanGain = predictedVariance / (predictedVariance + QUANTIZATION_STEP);

  // Update estimate and variance
  kalmanEstimate += kalmanGain * (rawADC - kalmanEstimate);
  predictedVariance = (1 - kalmanGain) * predictedVariance;

  return kalmanEstimate;
}

// Measure voltage using ADC and Kalman filter
double measureVoltage() {
  double sum = 0;
  for (int i = 0; i < 10; i++) {
    double rawADC = analogRead(MEASURE_PIN);
    sum += adaptiveKalmanFilter(rawADC * (VCC / ADC_RESOLUTION));
    delayMicroseconds(50);
  }
  return sum / 10;
}

// Richardson extrapolation for numerical refinement
double refineWithRichardson(double coarse, double fine, double ratio) {
  return fine + (fine - coarse) / (pow(ratio, 2) - 1);
}

// Solve ODE for charging with refined numerical methods
double solveODE(double vc, double R, double dt) {
  auto f = [&](double v) {
    return (VCC - v) / (R * CAPACITANCE);
  };

  double k1 = f(vc);
  double k2 = f(vc + 0.5 * dt * k1);
  double k3 = f(vc + 0.5 * dt * k2);
  double k4 = f(vc + dt * k3);

  double coarse = vc + dt * k1;         // Coarse estimate
  double fine = vc + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4); // Fine estimate

  return refineWithRichardson(coarse, fine, 2.0); // Richardson refinement
}

// Adaptive pulse charging function
double chargeCapacitor(double targetVoltage, double &lastResistance) {
  pinMode(CHARGE_PIN, OUTPUT);
  digitalWrite(CHARGE_PIN, LOW); // Discharge capacitor
  delay(10);
  pinMode(CHARGE_PIN, INPUT);

  double vc = 0;       // Initial capacitor voltage
  double t = 0;        // Time accumulator
  double dt = pulseWidthUs * 1e-6; // Initial time step
  std::vector<double> times, voltages;

  // Begin charging loop
  while (vc < targetVoltage) {
    // Generate precise charge pulse
    uint32_t pulseCycles = (uint32_t)(pulseWidthUs * (float)ESP.getCpuFreqMHz());
    pinMode(CHARGE_PIN, OUTPUT);
    digitalWrite(CHARGE_PIN, HIGH);
    delayCycles(pulseCycles);
    digitalWrite(CHARGE_PIN, LOW);
    pinMode(CHARGE_PIN, INPUT);

    // Measure voltage
    double measuredVoltage = measureVoltage();
    voltages.push_back(measuredVoltage);
    times.push_back(t);

    // Solve ODE with refined numerical methods
    double theoreticalVoltage = solveODE(vc, lastResistance, dt);

    // Estimate resistance with linear regression
    double deltaV = measuredVoltage - vc;
    double theoreticalDeltaV = theoreticalVoltage - vc;
    if (fabs(deltaV - theoreticalDeltaV) > QUANTIZATION_STEP) {
      lastResistance *= deltaV / theoreticalDeltaV;
    }

    // Update for next iteration
    vc = measuredVoltage;
    t += dt;
    pulseWidthUs = constrain(pulseWidthUs * (1 + 0.1 * (deltaV - theoreticalDeltaV)), MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);
    dt = pulseWidthUs * 1e-6;
  }

  // Nonlinear curve fitting
  double sumT = 0, sumT2 = 0, sumV = 0, sumTV = 0;
  for (size_t i = 0; i < times.size(); i++) {
    sumT += times[i];
    sumT2 += times[i] * times[i];
    sumV += log(VCC - voltages[i]);
    sumTV += times[i] * log(VCC - voltages[i]);
  }
  double slope = (times.size() * sumTV - sumT * sumV) / (times.size() * sumT2 - sumT * sumT);
  lastResistance = -1 / (slope * CAPACITANCE);

  return lastResistance;
}

void setup() {
  Serial.begin(115200);
  pinMode(CHARGE_PIN, OUTPUT);
  digitalWrite(CHARGE_PIN, LOW);
}

void loop() {
  double targetVoltage = VCC * 0.75; // 75% of VCC
  static double lastResistance = 1e3; // Initial guess: 1 kOhm

  double resistance = chargeCapacitor(targetVoltage, lastResistance);

  Serial.print("Estimated Resistance: ");
  Serial.println(resistance, 8);

  delay(1000);
}
