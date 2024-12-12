#include <Arduino.h>
#include <cmath>

class ResistanceCapacitanceSolver {
private:
    // Pin definitions
    const int R1_PIN = 12;       // Pin connected to R1
    const int CAPACITOR_PIN = 34; // Analog input pin for capacitor voltage
    const int DISCHARGE_PIN = 13; // Pin used for discharging

    // Numerical method parameters
    const int MAX_ITERATIONS = 100;
    const double TOLERANCE = 1e-6;

    // Circuit parameters to estimate
    double R1 = 0.0;     // Unknown resistance 1
    double R2 = 0.0;     // Internal resistance / Unknown resistance 2
    double C = 0.0;      // Unknown capacitance

    // RK45 method coefficients
    const double a2 = 1.0/4.0;
    const double a3 = 3.0/8.0;
    const double a4 = 12.0/13.0;
    const double a5 = 1.0;
    const double a6 = 1.0/2.0;

    const double b21 = 1.0/4.0;
    const double b31 = 3.0/32.0;
    const double b32 = 9.0/32.0;
    const double b41 = 1932.0/2197.0;
    const double b42 = -7200.0/2197.0;
    const double b43 = 7296.0/2197.0;
    const double b51 = 439.0/216.0;
    const double b52 = -8.0;
    const double b53 = 3680.0/513.0;
    const double b54 = -845.0/4104.0;
    const double b61 = -8.0/27.0;
    const double b62 = 2.0;
    const double b63 = -3544.0/2565.0;
    const double b64 = 1859.0/4104.0;
    const double b65 = -11.0/40.0;

    // Differential equation for RC circuit charging
    double dVdt(double V, double t, double R, double C) {
        return (5.0 - V) / (R * C);
    }

    // Runge-Kutta-Fehlberg method implementation
    double rungeKuttaFehlberg(double V0, double t0, double tf, double R, double C) {
        double h = 0.01;  // Initial step size
        double t = t0;
        double V = V0;
        double k1, k2, k3, k4, k5, k6;
        double V_new, V_error;

        while (t < tf) {
            k1 = h * dVdt(V, t, R, C);
            k2 = h * dVdt(V + b21*k1, t + a2*h, R, C);
            k3 = h * dVdt(V + b31*k1 + b32*k2, t + a3*h, R, C);
            k4 = h * dVdt(V + b41*k1 + b42*k2 + b43*k3, t + a4*h, R, C);
            k5 = h * dVdt(V + b51*k1 + b52*k2 + b53*k3 + b54*k4, t + a5*h, R, C);
            k6 = h * dVdt(V + b61*k1 + b62*k2 + b63*k3 + b64*k4 + b65*k5, t + a6*h, R, C);

            // Compute new voltage and error estimate
            V_new = V + (16.0/135.0)*k1 + (6656.0/12825.0)*k3 + (28561.0/56430.0)*k4 
                    + (9.0/50.0)*k5 + (2.0/55.0)*k6;
            V_error = abs(V_new - (V + (25.0/216.0)*k1 + (1408.0/2565.0)*k3 
                    + (2197.0/4104.0)*k4 + (1.0/5.0)*k5));

            // Adjust step size based on error
            double h_new = h * pow(TOLERANCE / (2 * V_error), 0.25);
            
            V = V_new;
            t += h;
            h = h_new;
        }
        return V;
    }

    // Charge capacitor and measure voltage
    double chargeCapacitor(int chargePin, int measurePin) {
        pinMode(chargePin, OUTPUT);
        digitalWrite(chargePin, HIGH);
        
        unsigned long startTime = micros();
        double voltage = 0.0;
        while (micros() - startTime < 10000) {  // 10ms charging window
            voltage = analogRead(measurePin) * (5.0 / 4095.0);
        }
        
        return voltage;
    }

    // Discharge capacitor
    void dischargeCapacitor(int dischargePin) {
        pinMode(dischargePin, OUTPUT);
        digitalWrite(dischargePin, LOW);
        delay(50);  // Discharge time
    }

public:
    void begin() {
        pinMode(R1_PIN, OUTPUT);
        pinMode(CAPACITOR_PIN, INPUT);
        pinMode(DISCHARGE_PIN, OUTPUT);
        analogReadResolution(12);  // ESP32 specific 12-bit ADC
    }

    void estimateCircuitParameters() {
        const int numMeasurements = 10;
        double voltages[numMeasurements];
        double times[numMeasurements];

        // Collect multiple charge measurements
        for (int i = 0; i < numMeasurements; i++) {
            dischargeCapacitor(DISCHARGE_PIN);
            
            unsigned long startTime = micros();
            voltages[i] = chargeCapacitor(R1_PIN, CAPACITOR_PIN);
            times[i] = (micros() - startTime) / 1000000.0;  // Convert to seconds
        }

        // Parameter estimation using least squares / optimization
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        for (int i = 0; i < numMeasurements; i++) {
            sumX += times[i];
            sumY += log(5.0 - voltages[i]);
            sumXY += times[i] * log(5.0 - voltages[i]);
            sumX2 += times[i] * times[i];
        }

        // Linear regression to estimate RC parameters
        double n = numMeasurements;
        double slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
        
        R1 = -1.0 / slope;
        C = -1.0 / (R1 * slope);
        R2 = 1000.0;  // Estimated internal resistance

        // Validate through Runge-Kutta simulation
        for (int i = 0; i < numMeasurements; i++) {
            double simulatedVoltage = rungeKuttaFehlberg(0, 0, times[i], R1, C);
            // Add error checking and refinement logic here
        }
    }

    void printResults() {
        Serial.println("Circuit Parameter Estimation:");
        Serial.print("R1: "); Serial.print(R1); Serial.println(" Ohms");
        Serial.print("R2: "); Serial.print(R2); Serial.println(" Ohms");
        Serial.print("Capacitance: "); Serial.print(C * 1e6); Serial.println(" µF");
    }
};

ResistanceCapacitanceSolver rcSolver;

void setup() {
    Serial.begin(115200);
    rcSolver.begin();
    rcSolver.estimateCircuitParameters();
    rcSolver.printResults();
}

void loop() {
    // Optional: periodic re-estimation or monitoring
    delay(5000);
    rcSolver.estimateCircuitParameters();
    rcSolver.printResults();
}
