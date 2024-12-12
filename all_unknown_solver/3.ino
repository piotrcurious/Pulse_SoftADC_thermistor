#include <Arduino.h>
#include <vector>
#include <numeric>
#include <cmath>

class AdaptivePulseRCSolver {
private:
    // Pin Definitions
    static const int R1_PIN = 12;
    static const int R2_PIN = 13;
    static const int CAPACITOR_PIN = 34;

    // Pulse Characteristics
    struct PulseParams {
        uint32_t width;       // Pulse width in microseconds
        uint32_t period;      // Total period between pulses
        uint8_t dutyCycle;    // Duty cycle percentage
    };

    // Circuit Parameters
    struct CircuitParams {
        double R1;            // First resistance
        double R2;            // Second resistance
        double C;             // Capacitance
        double voltageRead;   // Last measured voltage
    };

    CircuitParams currentParams = {10000.0, 10000.0, 1e-6, 0.0};
    
    // Adaptive Pulse Configuration
    PulseParams pulseCfg = {50, 1000, 50};  // Initial pulse configuration
    
    // Optimization Parameters
    const int MAX_ITERATIONS = 100;
    const double CONVERGENCE_THRESHOLD = 1e-6;
    const double LEARNING_RATE = 0.01;

    // Noise and Measurement Configuration
    const int NUM_SAMPLES = 10;
    const double SUPPLY_VOLTAGE = 3.3;

    // Reset pin to input state
    void resetPinToInput(int pin) {
        pinMode(pin, INPUT);
        digitalWrite(pin, LOW);
    }

    // Generate an adaptive pulse with variable characteristics
    double generateAdaptivePulse(int chargePin, int measurePin, PulseParams& pulse) {
        // Set up pulse generation
        pinMode(chargePin, OUTPUT);
        
        // Adaptive pulse generation
        unsigned long startTime = micros();
        uint32_t pulseOnTime = (pulse.width * pulse.dutyCycle) / 100;
        
        // Generate pulse
        digitalWrite(chargePin, HIGH);
        delayMicroseconds(pulseOnTime);
        digitalWrite(chargePin, LOW);
        
        // Wait for stabilization
        delayMicroseconds(10);
        
        // Read voltage
        double voltage = analogRead(measurePin) * (SUPPLY_VOLTAGE / 4095.0);
        
        // Reset pin
        resetPinToInput(chargePin);
        
        return voltage;
    }

    // Compute derivative of RC circuit response
    double computeRCDerivative(const CircuitParams& params, double voltage) {
        // Compute time constant and expected voltage change
        double tau = params.R1 * params.R2 * params.C / (params.R1 + params.R2);
        double expectedVoltageChange = (SUPPLY_VOLTAGE - voltage) * (1.0 - exp(-pulseCfg.width / (1e6 * tau)));
        
        return expectedVoltageChange;
    }

    // Adaptive parameter update using gradient descent
    void updateCircuitParameters(double voltageRead) {
        // Compute sensitivities
        double dR1 = computeRCDerivative({currentParams.R1 * 1.01, currentParams.R2, currentParams.C, 0}, voltageRead);
        double dR2 = computeRCDerivative({currentParams.R1, currentParams.R2 * 1.01, currentParams.C, 0}, voltageRead);
        double dC = computeRCDerivative({currentParams.R1, currentParams.R2, currentParams.C * 1.01, 0}, voltageRead);

        // Update parameters with adaptive learning
        currentParams.R1 -= LEARNING_RATE * (dR1 - computeRCDerivative(currentParams, voltageRead));
        currentParams.R2 -= LEARNING_RATE * (dR2 - computeRCDerivative(currentParams, voltageRead));
        currentParams.C -= LEARNING_RATE * (dC - computeRCDerivative(currentParams, voltageRead));

        // Constrain parameters
        currentParams.R1 = constrain(currentParams.R1, 100.0, 1000000.0);
        currentParams.R2 = constrain(currentParams.R2, 100.0, 1000000.0);
        currentParams.C = constrain(currentParams.C, 1e-9, 1e-3);
    }

    // Adaptive pulse configuration strategy
    void adaptPulseCharacteristics(double convergenceError) {
        // Dynamically adjust pulse parameters based on convergence
        if (convergenceError > 0.1) {
            // High error: increase pulse width and frequency
            pulseCfg.width = min(pulseCfg.width * 1.2, 500U);
            pulseCfg.period = max(pulseCfg.period / 1.1, 100U);
            pulseCfg.dutyCycle = min(pulseCfg.dutyCycle + 5, 90U);
        } else if (convergenceError < 0.01) {
            // Low error: decrease pulse width and frequency
            pulseCfg.width = max(pulseCfg.width / 1.2, 10U);
            pulseCfg.period = min(pulseCfg.period * 1.1, 5000U);
            pulseCfg.dutyCycle = max(pulseCfg.dutyCycle - 5, 10U);
        }
    }

public:
    void begin() {
        Serial.begin(115200);
        analogReadResolution(12);
        pinMode(R1_PIN, INPUT);
        pinMode(R2_PIN, INPUT);
        pinMode(CAPACITOR_PIN, INPUT);
    }

    void estimateParameters() {
        std::vector<double> voltageReadingsR1, voltageReadingsR2;
        double convergenceError = 1.0;

        for (int iteration = 0; iteration < MAX_ITERATIONS && convergenceError > CONVERGENCE_THRESHOLD; ++iteration) {
            // Reset voltage readings
            voltageReadingsR1.clear();
            voltageReadingsR2.clear();

            // Measure through R1 pin
            for (int i = 0; i < NUM_SAMPLES; ++i) {
                double voltage = generateAdaptivePulse(R1_PIN, CAPACITOR_PIN, pulseCfg);
                voltageReadingsR1.push_back(voltage);
            }

            // Measure through R2 pin
            for (int i = 0; i < NUM_SAMPLES; ++i) {
                double voltage = generateAdaptivePulse(R2_PIN, CAPACITOR_PIN, pulseCfg);
                voltageReadingsR2.push_back(voltage);
            }

            // Compute average voltage
            double avgVoltageR1 = std::accumulate(voltageReadingsR1.begin(), voltageReadingsR1.end(), 0.0) / voltageReadingsR1.size();
            double avgVoltageR2 = std::accumulate(voltageReadingsR2.begin(), voltageReadingsR2.end(), 0.0) / voltageReadingsR2.size();

            // Update circuit parameters
            updateCircuitParameters(avgVoltageR1);
            updateCircuitParameters(avgVoltageR2);

            // Compute convergence error
            convergenceError = abs(avgVoltageR1 - avgVoltageR2) / SUPPLY_VOLTAGE;

            // Adapt pulse characteristics
            adaptPulseCharacteristics(convergenceError);
        }
    }

    void printResults() {
        Serial.println("\nEstimated Circuit Parameters:");
        Serial.print("R1: "); Serial.print(currentParams.R1, 2); Serial.println(" Ω");
        Serial.print("R2: "); Serial.print(currentParams.R2, 2); Serial.println(" Ω");
        Serial.print("Capacitance: "); Serial.print(currentParams.C * 1e6, 3); Serial.println(" µF");
        
        Serial.println("\nFinal Pulse Configuration:");
        Serial.print("Pulse Width: "); Serial.print(pulseCfg.width); Serial.println(" µs");
        Serial.print("Pulse Period: "); Serial.print(pulseCfg.period); Serial.println(" µs");
        Serial.print("Duty Cycle: "); Serial.print(pulseCfg.dutyCycle); Serial.println(" %");
    }
};

AdaptivePulseRCSolver rcSolver;

void setup() {
    rcSolver.begin();
    rcSolver.estimateParameters();
    rcSolver.printResults();
}

void loop() {
    // Periodic re-estimation
    delay(5000);
    rcSolver.estimateParameters();
    rcSolver.printResults();
}
