
// Define the pins and constants
const int chargePin = 13; // The pin that charges the capacitor
const int measurePin = A0; // The pin that measures the capacitor voltage
const int ledPin = 12; // The pin that indicates the charging status
const float capValue = 0.0001; // The value of the capacitor in farads
const float threshold = 4.0; // The voltage threshold in volts

// Declare the variables
int pulseCount = 0; // The number of pulses sent to the capacitor
float capVoltage = 0.0; // The voltage across the capacitor
float resValue = 0.0; // The resistance value in ohms

void setup() {
  // Initialize the pins
  pinMode(chargePin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(measurePin, INPUT);
  digitalWrite(chargePin, LOW);
  digitalWrite(ledPin, LOW);
  
  // Initialize the serial monitor
  Serial.begin(9600);
}

void loop() {
  // Charge the capacitor with discrete pulses
  digitalWrite(chargePin, HIGH); // Send a high pulse
  delay(10); // Wait for 10 milliseconds
  digitalWrite(chargePin, LOW); // Send a low pulse
  pulseCount++; // Increment the pulse count
  
  // Measure the capacitor voltage
  capVoltage = analogRead(measurePin) * (5.0 / 1023.0); // Convert the analog reading to volts
  
  // Check if the capacitor voltage is above the threshold
  if (capVoltage >= threshold) {
    // Turn on the LED to indicate charging is done
    digitalWrite(ledPin, HIGH);
    
    // Calculate the resistance value using the formula: R = (t / C) / ln(Vc / V)
    resValue = (pulseCount * 10.0 / 1000.0 / capValue) / log(capVoltage / 5.0);
    
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
    delay(5000); // Wait for 5 seconds
    
    // Turn off the LED to indicate ready for a new measurement
    digitalWrite(ledPin, LOW);
    
  }
}
