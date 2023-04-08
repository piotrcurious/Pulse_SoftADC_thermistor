
// Define the pins and constants
const int chargePin = 9; // The pin that charges the capacitor
const int measurePin = A0; // The pin that measures the capacitor voltage
const int threshold = 800; // The analog value that indicates the capacitor is charged
const float capValue = 0.01; // The known value of the capacitor in Farads
const float R1 = 10000; // The known value of the resistor in series with the capacitor in Ohms

// Define the variables
float pulseLength = 0.001; // The initial length of the charging pulse in seconds
int pulseCount = 0; // The number of pulses needed to charge the capacitor
float R2; // The unknown value of the resistor in series with the capacitor in Ohms
float error; // The error between the expected and actual discharge pulse

void setup() {
  // Set the charge pin as output and measure pin as input
  pinMode(chargePin, OUTPUT);
  pinMode(measurePin, INPUT);
  
  // Start the serial monitor
  Serial.begin(9600);
}

void loop() {
  // Reset the pulse count and error
  pulseCount = 0;
  error = 0;
  
  // Charge the capacitor with discrete pulses until it reaches the threshold
  while (analogRead(measurePin) < threshold) {
    digitalWrite(chargePin, HIGH); // Turn on the charge pin
    delay(pulseLength * 1000); // Wait for the pulse length in milliseconds
    digitalWrite(chargePin, LOW); // Turn off the charge pin
    pulseCount++; // Increment the pulse count
  }
  
  // Calculate the resistance of R2 using the formula R2 = (RC - R1) / N, where RC is the time constant and N is the number of pulses
  R2 = (capValue * log(1023.0 / (1023.0 - threshold)) - R1) / pulseCount;
  
  // Print the results to the serial monitor
  Serial.print("Pulse count: ");
  Serial.println(pulseCount);
  Serial.print("Pulse length: ");
  Serial.print(pulseLength * 1000);
  Serial.println(" ms");
  Serial.print("Resistance: ");
  Serial.print(R2);
  Serial.println(" Ohms");
  
  // Compute a verification discharge pulse that should discharge the capacitor by a small amount (10%)
  float dischargePulse = capValue * log(0.9) / (R1 + R2); // The expected length of the discharge pulse in seconds
  
  // Discharge the capacitor with a single pulse and measure the voltage drop
  digitalWrite(chargePin, HIGH); // Turn on the charge pin to reverse polarity
  delay(dischargePulse * 1000); // Wait for the discharge pulse length in milliseconds
  digitalWrite(chargePin, LOW); // Turn off the charge pin
  int voltageDrop = threshold - analogRead(measurePin); // The actual voltage drop in analog units
  
  // Calculate the error between the expected and actual voltage drop using delta-sigma method
  error = (voltageDrop - threshold * 0.1) / threshold;
  
  // Print the error to the serial monitor
  Serial.print("Error: ");
  Serial.println(error);
  
  // Adjust the pulse length based on the error to increase precision
  if (error > 0) {
    // The voltage drop was too large, so decrease the pulse length by a small factor (0.99)
    pulseLength *= 0.99;
  }
  else if (error < 0) {
    // The voltage drop was too small, so increase the pulse length by a small factor (1.01)
    pulseLength *= 1.01;
    
    }
    
    }
