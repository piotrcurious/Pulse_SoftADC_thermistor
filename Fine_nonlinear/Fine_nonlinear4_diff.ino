
#include <driver/timer.h>
#include <cmath>

// Define the pins and constants
const int chargePin = 13; // The pin that charges the capacitor
const int measurePin = A0; // The pin that measures the capacitor voltage
const int ledPin = 12; // The pin that indicates the charging status

// Precise measurement constants
const float capValue = 0.0001; // The value of the capacitor in farads
const float supplyVoltage = 5.0; // Supply voltage
const float threshold = 4.0; // The voltage threshold in volts
const uint32_t roughPulseCycles = 240000; // Rough pulse duration in CPU cycles (10ms at 24MHz)
const uint32_t finePulseCycles = 2400; // Fine pulse duration in CPU cycles (0.1ms at 24MHz)
const float ADC_RESOLUTION = 5.0 / 1023.0; // ADC voltage per bit
const int MAX_MEASUREMENTS = 200; // Maximum number of voltage measurements

// Measurement data storage
struct MeasurementPoint {
  float voltage;
  uint32_t timestamp;
};

class CapacitorChargeAnalyzer {
private:
  MeasurementPoint measurements[MAX_MEASUREMENTS];
  int measurementCount = 0;
  uint32_t startTimestamp = 0;

  // Nonlinear least squares method for parameter estimation
  bool estimateResistanceCapacitance(float& R, float& C) {
    if (measurementCount < 10) return false;

    // Initial guess using linear approximation
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    for (int i = 0; i < measurementCount; i++) {
      float t = (measurements[i].timestamp - startTimestamp) / 1000000.0; // Convert to seconds
      float lnTerm = log(supplyVoltage / (supplyVoltage - measurements[i].voltage));
      
      sumX += t;
      sumY += lnTerm;
      sumXY += t * lnTerm;
      sumX2 += t * t;
    }

    float n = measurementCount;
    float linearCovariance = (sumXY - sumX * sumY / n) / (sumX2 - sumX * sumX / n);
    
    // Initial estimates
    float initialR = -1.0 / linearCovariance;
    float initialC = capValue; // Use predefined capacitance as initial guess

    // Levenberg-Marquardt optimization
    const int maxIterations = 50;
    const float lambdaInit = 0.001;
    float lambda = lambdaInit;
    float prevError = INFINITY;

    for (int iter = 0; iter < maxIterations; iter++) {
      // Compute Jacobian and error
      float sumError = 0;
      Eigen::MatrixXf J(measurementCount, 2);
      Eigen::VectorXf errors(measurementCount);

      for (int i = 0; i < measurementCount; i++) {
        float t = (measurements[i].timestamp - startTimestamp) / 1000000.0;
        float predVoltage = supplyVoltage * (1 - exp(-t / (initialR * initialC)));
        float error = measurements[i].voltage - predVoltage;
        
        // Compute partial derivatives
        float dVdR = supplyVoltage * exp(-t / (initialR * initialC)) * (t / (initialR * initialC * initialR));
        float dVdC = supplyVoltage * exp(-t / (initialR * initialC)) * (t / (initialR * initialC * initialC));
        
        J(i, 0) = dVdR;
        J(i, 1) = dVdC;
        errors(i) = error;
        
        sumError += error * error;
      }

      // Normal equations
      Eigen::MatrixXf JT = J.transpose();
      Eigen::MatrixXf JTJ = JT * J;
      Eigen::VectorXf JTe = JT * errors;

      // Damped least squares
      for (int j = 0; j < 2; j++) {
        JTJ(j,j) *= (1.0 + lambda);
      }

      Eigen::VectorXf delta = JTJ.ldlt().solve(JTe);

      // Update parameters
      float newR = initialR + delta(0);
      float newC = initialC + delta(1);

      // Compute new error
      float newSumError = 0;
      for (int i = 0; i < measurementCount; i++) {
        float t = (measurements[i].timestamp - startTimestamp) / 1000000.0;
        float predVoltage = supplyVoltage * (1 - exp(-t / (newR * newC)));
        float error = measurements[i].voltage - predVoltage;
        newSumError += error * error;
      }

      // Adjust lambda based on error reduction
      if (newSumError < prevError) {
        initialR = newR;
        initialC = newC;
        lambda /= 10.0;
        prevError = newSumError;
      } else {
        lambda *= 10.0;
      }

      // Convergence check
      if (abs(prevError - newSumError) / prevError < 1e-6) {
        break;
      }
    }

    R = initialR;
    C = initialC;
    return true;
  }

public:
  void reset() {
    measurementCount = 0;
    startTimestamp = 0;
  }

  void addMeasurement(float voltage) {
    if (measurementCount < MAX_MEASUREMENTS) {
      if (measurementCount == 0) {
        startTimestamp = esp_timer_get_time();
      }
      
      measurements[measurementCount] = {
        voltage, 
        esp_timer_get_time()
      };
      measurementCount++;
    }
  }

  bool computeResistance(float& resistance, float& capacitance) {
    return estimateResistanceCapacitance(resistance, capacitance);
  }
};

// Global instances
CapacitorChargeAnalyzer analyzer;
float resValue = 0.0;
float capValue = 0.0;
bool measurementComplete = false;

void setup() {
  // Initialize pins
  pinMode(ledPin, OUTPUT);
  pinMode(chargePin, OUTPUT);
  pinMode(measurePin, INPUT);
  
  // Initialize serial communication
  Serial.begin(115200);
  
  // Initialize ESP32 timer
  esp_timer_init();
}

float measureCapacitorVoltage() {
  // Take multiple readings and average them
  float voltageSum = 0;
  const int numReadings = 10;
  
  for (int i = 0; i < numReadings; i++) {
    voltageSum += analogRead(measurePin) * ADC_RESOLUTION;
    delayMicroseconds(10);
  }
  
  return voltageSum / numReadings;
}

void chargeCapacitor() {
  // Reset analyzer
  analyzer.reset();
  
  // Rough charging phase
  digitalWrite(chargePin, HIGH);
  delayCycles(roughPulseCycles);
  
  // Fine charging and measurement phase
  for (int i = 0; i < MAX_MEASUREMENTS; i++) {
    // Alternate charging and discharging
    digitalWrite(chargePin, (i % 2 == 0) ? HIGH : LOW);
    
    // Short precise pulse
    delayCycles(finePulseCycles);
    
    // Measure and store voltage
    float voltage = measureCapacitorVoltage();
    analyzer.addMeasurement(voltage);
    
    // Stop if near threshold
    if (abs(voltage - threshold) < ADC_RESOLUTION) {
      break;
    }
  }
  
  // Compute resistance
  measurementComplete = analyzer.computeResistance(resValue, capValue);
}

void loop() {
  if (!measurementComplete) {
    chargeCapacitor();
    
    if (measurementComplete) {
      // Turn on LED to indicate completion
      digitalWrite(ledPin, HIGH);
      
      // Print results
      Serial.println("Resistance Measurement Complete:");
      Serial.print("Resistance: ");
      Serial.print(resValue);
      Serial.println(" Ω");
      Serial.print("Actual Capacitance: ");
      Serial.print(capValue * 1e6);
      Serial.println(" µF");
      
      // Wait before next measurement
      delay(5000);
      digitalWrite(ledPin, LOW);
    }
  }
}

// Utility function for precise cycle delay
uint32_t delayCycles(uint32_t cycles) {
  uint32_t start = esp_timer_get_time() * 240;
  uint32_t end = start + cycles;
  while ((esp_timer_get_time() * 240) < end) {
    // Busy wait
  }
  return cycles;
}
