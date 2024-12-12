#include <Arduino.h>

// Pin definitions
const int CHARGE_PIN = 5;    // Digital pin to pulse for charging
const int MEASURE_PIN = 34;  // Analog pin to measure capacitor voltage

// Known parameters
const float CAPACITANCE = 1e-6; // Known capacitance in Farads
const float VCC = 3.3;          // Supply voltage in Volts
const float ADC_RESOLUTION = 4096.0; // 12-bit ADC resolution

// RKF45 method parameters
const float h_init = 1e-4; // Initial time step
const float tol = 1e-6;    // Tolerance for the solution

// Function to simulate capacitor charging
float chargingODE(float t, float vc, float R) {
  return (VCC - vc) / (R * CAPACITANCE);
}

// Runge-Kutta-Fehlberg method to solve ODE
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

// Error correction using Galois Fields
float errorCorrectADC(float rawADC) {
  // Example of a basic correction using finite field (GF(2^12))
  int gf_poly = 0xFFF; // Galois field primitive polynomial
  int correctedADC = (int)rawADC ^ gf_poly;
  return correctedADC * (VCC / ADC_RESOLUTION);
}

void setup() {
  Serial.begin(115200);

  pinMode(CHARGE_PIN, OUTPUT);
  digitalWrite(CHARGE_PIN, LOW);
}

void loop() {
  // Pulse to charge the capacitor
  digitalWrite(CHARGE_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(CHARGE_PIN, LOW);

  // Measure the capacitor voltage
  int rawADC = analogRead(MEASURE_PIN);
  float correctedVoltage = errorCorrectADC(rawADC);

  // Estimate resistance using numerical methods
  float targetTime = 0.001; // Example: solve for time = 1 ms
  float estimatedResistance = 1.0; // Initial guess

  float vc_simulated = solveChargingRKF45(estimatedResistance, targetTime);
  while (fabs(vc_simulated - correctedVoltage) > tol) {
    estimatedResistance *= (correctedVoltage / vc_simulated);
    vc_simulated = solveChargingRKF45(estimatedResistance, targetTime);
  }

  Serial.print("Estimated Resistance: ");
  Serial.println(estimatedResistance, 2);
  delay(1000);
}
