#include <Arduino.h>

// Pin definitions
const int CHARGE_PIN = 5;    // Digital pin to pulse for charging
const int MEASURE_PIN = 34;  // Analog pin to measure capacitor voltage

// Known parameters
const float CAPACITANCE = 1e-6; // Known capacitance in Farads
const float VCC = 3.3;          // Supply voltage in Volts
const int NUM_SAMPLES = 10;     // Number of samples to average
const float ADC_RESOLUTION = 4096.0; // 12-bit ADC resolution

// RKF45 parameters
const float h_init = 1e-5; // Initial time step
const float tol = 1e-6;    // Tolerance for the solution

// Differential equation for capacitor charging with quantization correction
float chargingODE(float t, float vc, float R) {
  float quantization_error = (VCC / ADC_RESOLUTION) / 2; // ADC quantization step
  return (VCC - vc + quantization_error) / (R * CAPACITANCE);
}

// Runge-Kutta-Fehlberg method
float solveChargingRKF45(float R, float timeLimit) {
  float vc = 0; // Initial capacitor voltage
  float t = 0;  // Initial time
  float h = h_init;

  while (t < timeLimit) {
    float k1 = h * chargingODE(t, vc, R);
    float k2 = h * chargingODE(t + h / 4, vc + k1 / 4, R);
    float k3 = h * chargingODE(t + 3 * h / 8, vc + 3 * k1 / 32 + 9 * k2 / 32, R);
    float k4 = h * chargingODE(t + 12 * h / 13, vc + 1932 * k1 / 2197 - 7200 * k2 / 2197 + 7296 * k3 / 2197, R);
    float k5 = h * chargingODE(t + h, vc + 439 * k1 / 216 - 8 * k2 + 3680 * k3 / 513 - 845 * k4 / 4104, R);
    float k6 = h * chargingODE(t + h / 2, vc - 8 * k1 / 27 + 2 * k2 - 3544 * k3 / 2565 + 1859 * k4 / 4104 - 11 * k5 / 40, R);

    float vc_next = vc + 16 * k1 / 135 + 6656 * k3 / 12825 + 28561 * k4 / 56430 - 9 * k5 / 50 + 2 * k6 / 55;
    float vc_error = fabs(vc_next - vc);

    if (vc_error < tol) {
      t += h;
      vc = vc_next;
    }

    h *= 0.9 * pow(tol / vc_error, 0.2);
  }

  return vc;
}

// Average ADC measurements for noise reduction
float averageADC(int pin) {
  float sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(50);
  }
  return sum / NUM_SAMPLES;
}

// Discharge the capacitor
void dischargeCapacitor(int chargePin) {
  pinMode(chargePin, OUTPUT);
  digitalWrite(chargePin, LOW);
  delay(10); // Ensure full discharge
  pinMode(chargePin, INPUT); // Tri-state to minimize leakage
}

void setup() {
  Serial.begin(115200);
  pinMode(CHARGE_PIN, OUTPUT);
  digitalWrite(CHARGE_PIN, LOW);
}

void loop() {
  // Fully discharge the capacitor before starting
  dischargeCapacitor(CHARGE_PIN);

  // Charge the capacitor
  digitalWrite(CHARGE_PIN, HIGH);
  delayMicroseconds(10); // Short pulse to start charging
  digitalWrite(CHARGE_PIN, LOW);
  pinMode(CHARGE_PIN, INPUT); // Tri-state mode after charging

  // Measure capacitor voltage
  float rawADC = averageADC(MEASURE_PIN);
  float measuredVoltage = rawADC * (VCC / ADC_RESOLUTION);

  // Estimate resistance
  float targetTime = 0.001; // Solve for time = 1 ms
  float estimatedResistance = 1.0; // Initial guess

  float vc_simulated = solveChargingRKF45(estimatedResistance, targetTime);
  while (fabs(vc_simulated - measuredVoltage) > tol) {
    estimatedResistance *= (measuredVoltage / vc_simulated);
    vc_simulated = solveChargingRKF45(estimatedResistance, targetTime);
  }

  Serial.print("Measured Voltage: ");
  Serial.println(measuredVoltage, 4);
  Serial.print("Estimated Resistance: ");
  Serial.println(estimatedResistance, 2);

  delay(1000); // Wait before the next cycle
}
