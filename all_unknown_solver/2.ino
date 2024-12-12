#include <Arduino.h>
#include <vector>
#include <Eigen.h>  // Linear algebra library for matrix operations

class AdvancedRCParameterSolver {
private:
    // Pin definitions
    static const int R1_PIN = 12;
    static const int R2_PIN = 13;
    static const int CAPACITOR_PIN = 34;

    // Circuit parameters to estimate
    double R1 = 10000.0;  // Initial guess
    double R2 = 10000.0;  // Initial guess
    double C = 1e-6;      // Initial guess for capacitance

    // Measurement and convergence parameters
    const int MAX_ITERATIONS = 50;
    const double CONVERGENCE_THRESHOLD = 1e-6;
    const int MEASUREMENT_SAMPLES = 10;
    const double SUPPLY_VOLTAGE = 3.3;  // ESP32 typical supply voltage

    // Reset pin to input state after operation
    void resetPinToInput(int pin) {
        pinMode(pin, INPUT);
        digitalWrite(pin, LOW);
    }

    // Charge capacitor through a specific pin and measure voltage
    double chargeThroughPin(int chargePin, int measurePin) {
        // Set charge pin to output and pull high
        pinMode(chargePin, OUTPUT);
        digitalWrite(chargePin, HIGH);
        
        // Measure voltage
        delayMicroseconds(100);  // Short charging pulse
        double voltage = analogRead(measurePin) * (SUPPLY_VOLTAGE / 4095.0);
        
        // Reset pins
        resetPinToInput(chargePin);
        return voltage;
    }

    // Discharge capacitor through a specific pin
    void dischargeThroughPin(int dischargePin) {
        pinMode(dischargePin, OUTPUT);
        digitalWrite(dischargePin, LOW);
        delayMicroseconds(50);  // Short discharge pulse
        resetPinToInput(dischargePin);
    }

    // RC Circuit Differential Equation
    Eigen::VectorXd computeRCDifferentialEquation(
        const Eigen::VectorXd& currentParams, 
        const std::vector<double>& voltages
    ) {
        double r1 = currentParams(0);
        double r2 = currentParams(1);
        double c = currentParams(2);

        Eigen::VectorXd derivatives(3);
        derivatives.setZero();

        // Iterative parameter update using sensitivity analysis
        for (size_t i = 1; i < voltages.size(); ++i) {
            double dt = 0.001;  // Time step
            double voltage_prev = voltages[i-1];
            double voltage_curr = voltages[i];

            // Compute voltage change rate
            double dVdt = (voltage_curr - voltage_prev) / dt;

            // Sensitivity derivatives
            derivatives(0) += abs(dVdt + voltage_curr / (r1 * c));
            derivatives(1) += abs(dVdt + voltage_curr / (r2 * c));
            derivatives(2) += abs(dVdt + voltage_curr / (r1 * r2 * c * c));
        }

        // Normalize derivatives
        derivatives /= derivatives.norm();
        return derivatives;
    }

    // Iterative parameter estimation
    void iterativeParameterEstimation() {
        Eigen::VectorXd params(3);
        params << R1, R2, C;

        std::vector<double> voltagesR1, voltagesR2;

        for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
            // Charge and measure through R1 pin
            voltagesR1.clear();
            for (int i = 0; i < MEASUREMENT_SAMPLES; ++i) {
                dischargeThroughPin(R2_PIN);
                double voltage = chargeThroughPin(R1_PIN, CAPACITOR_PIN);
                voltagesR1.push_back(voltage);
            }

            // Charge and measure through R2 pin
            voltagesR2.clear();
            for (int i = 0; i < MEASUREMENT_SAMPLES; ++i) {
                dischargeThroughPin(R1_PIN);
                double voltage = chargeThroughPin(R2_PIN, CAPACITOR_PIN);
                voltagesR2.push_back(voltage);
            }

            // Compute parameter update
            Eigen::VectorXd r1Derivatives = computeRCDifferentialEquation(params, voltagesR1);
            Eigen::VectorXd r2Derivatives = computeRCDifferentialEquation(params, voltagesR2);

            // Combine derivative information
            Eigen::VectorXd combinedDerivatives = (r1Derivatives + r2Derivatives) / 2.0;

            // Update parameters with adaptive learning rate
            double learningRate = 0.1 / (iteration + 1);
            params -= learningRate * combinedDerivatives;

            // Constrain parameters to reasonable ranges
            params(0) = constrain(params(0), 100.0, 1000000.0);  // R1: 100Ω to 1MΩ
            params(1) = constrain(params(1), 100.0, 1000000.0);  // R2: 100Ω to 1MΩ
            params(2) = constrain(params(2), 1e-9, 1e-3);        // C: 1nF to 1mF

            // Check convergence
            if ((combinedDerivatives.array().abs() < CONVERGENCE_THRESHOLD).all()) {
                break;
            }
        }

        // Update class variables
        R1 = params(0);
        R2 = params(1);
        C = params(2);
    }

public:
    void begin() {
        Serial.begin(115200);
        pinMode(R1_PIN, INPUT);
        pinMode(R2_PIN, INPUT);
        pinMode(CAPACITOR_PIN, INPUT);
        analogReadResolution(12);
    }

    void estimateParameters() {
        iterativeParameterEstimation();
    }

    void printResults() {
        Serial.println("Estimated Circuit Parameters:");
        Serial.print("R1: "); Serial.print(R1, 2); Serial.println(" Ω");
        Serial.print("R2: "); Serial.print(R2, 2); Serial.println(" Ω");
        Serial.print("Capacitance: "); Serial.print(C * 1e6, 3); Serial.println(" µF");
    }
};

AdvancedRCParameterSolver rcSolver;

void setup() {
    rcSolver.begin();
    rcSolver.estimateParameters();
    rcSolver.printResults();
}

void loop() {
    // Periodically re-estimate parameters
    delay(5000);
    rcSolver.estimateParameters();
    rcSolver.printResults();
}
