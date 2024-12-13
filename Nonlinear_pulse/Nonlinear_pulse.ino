
// Define the pins and constants
const int chargePin = 13; // The pin that charges the capacitor
const int measurePin = A0; // The pin that measures the capacitor voltage
const int ledPin = 12; // The pin that indicates the charging status
const float capValue = 0.0001; // The value of the capacitor in farads
const float threshold = 4.0; // The voltage threshold in volts
const uin32_t PULSE_LENGTH = 1 // lenght of the pulse
// Declare the variables
int pulseCount = 0; // The number of pulses sent to the capacitor
float capVoltage = 0.0; // The voltage across the capacitor
double resValue = 0.0; // The resistance value in ohms

void setup() {
  // Initialize the pins
  pinMode(chargePin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(measurePin, INPUT);
  digitalWrite(chargePin, LOW);
  digitalWrite(ledPin, LOW);
  
  // Initialize the serial monitor
  Serial.begin(115200);
}

void loop() {
  // Charge the capacitor with discrete pulses
  pinMode(chargePin, OUTPUT);
  digitalWrite(chargePin, HIGH); // Send a high pulse
  delay(PULSE_LENGTH); // Wait for pulse lenght
  pinMode(chargePin, INPUT);
  digitalWrite(chargePin, LOW); // end pulse and tristate
  pulseCount++; // Increment the pulse count
  
  // Measure the capacitor voltage
  capVoltage = analogRead(measurePin) * (5.0 / 1023.0); // Convert the analog reading to volts
  
  // Check if the capacitor voltage is above the threshold
  if (capVoltage >= threshold) {
    // Turn on the LED to indicate charging is done
    digitalWrite(ledPin, HIGH);
    pinMode(chargePin, OUTPUT);
    digitalWrite(chargePin, LOW); // discharge the capacitor 
    // Calculate the resistance value using the formula: R = (t / C) / ln(Vc / V)
    resValue = (pulseCount * (double)PULSE_LENGTH / 1000.0 / capValue) / log(capVoltage / 5.0);
    
    // Print the results to the serial monitor
    Serial.print("Pulse count: ");
    Serial.println(pulseCount);
    Serial.print("Capacitor voltage: ");
    Serial.print(capVoltage);
    Serial.println(" V");
    Serial.print("Resistance value: ");
    Serial.print(resValue);
    Serial.println(" ohms");
    
    // Reset the variables and wait for a new measurement
    pulseCount = 0;
    capVoltage = 0.0;
    resValue = 0.0;
    delay(1000); // Wait to make sure cap is discharged 
    
    // Turn off the LED to indicate ready for a new measurement
    digitalWrite(ledPin, LOW);
    
  }
}
