
// Define the pins and constants
const int chargePin = 13; // The pin that charges the capacitor
const int measurePin = A0; // The pin that measures the capacitor voltage
const int ledPin = 12; // The pin that indicates the charging status
const float capValue = 0.0001; // The value of the capacitor in farads
const float threshold = 4.0; // The voltage threshold in volts
const int roughPulse = 10; // The duration of the rough pulse in milliseconds
const int finePulse = 1; // The duration of the fine pulse in milliseconds

// Declare the variables
int roughPulseCount = 0; // The number of rough pulses sent to the capacitor
int finePulseCount = 0; // The number of fine pulses sent to the capacitor
float capVoltage = 0.0; // The voltage across the capacitor
float resValue = 0.0; // The resistance value in ohms
bool fineMode = false; // The flag that indicates the fine mode

void setup() {
  // Initialize the pins
  pinMode(ledPin, OUTPUT);
  pinMode(measurePin, INPUT);
  digitalWrite(ledPin, LOW);
  
  // Initialize the serial monitor
  Serial.begin(9600);
}

void loop() {
  // Charge the capacitor with discrete pulses
  if (fineMode) {
    // Use a fine pulse to charge and discharge the capacitor
    pinMode(chargePin, OUTPUT); // Set the charge pin to output mode
    digitalWrite(chargePin, HIGH); // Send a high pulse
    delayMicroseconds(finePulse); // Wait for a short time
    pinMode(chargePin, INPUT); // Set the charge pin to input mode
    delayMicroseconds(finePulse); // Wait for a short time
    finePulseCount++; // Increment the fine pulse count
    
    // Measure the capacitor voltage
    capVoltage = analogRead(measurePin) * (5.0 / 1023.0); // Convert the analog reading to volts
    
    // Check if the capacitor voltage is above or below the threshold
    if (capVoltage > threshold) {
      // Discharge the capacitor slightly by sending a low pulse
      pinMode(chargePin, OUTPUT); // Set the charge pin to output mode
      digitalWrite(chargePin, LOW);
      delayMicroseconds(finePulse);
      pinMode(chargePin, INPUT); // Set the charge pin to input mode
      capVoltage -= (5.0 / 1023.0); // Subtract a bit value from the voltage
    } else if (capVoltage < threshold) {
      // Charge the capacitor slightly by sending a high pulse
      pinMode(chargePin, OUTPUT); // Set the charge pin to output mode
      digitalWrite(chargePin, HIGH);
      delayMicroseconds(finePulse);
      pinMode(chargePin, INPUT); // Set the charge pin to input mode
      capVoltage += (5.0 / 1023.0); // Add a bit value to the voltage
    }
    
    // Check if the capacitor voltage is close to the threshold
    if (abs(capVoltage - threshold) < (5.0 / 1023.0)) {
      // Turn on the LED to indicate charging is done
      digitalWrite(ledPin, HIGH);
      
      // Calculate the total pulse time, including rough and fine pulses
      float totalPulseTime = (roughPulseCount * roughPulse + finePulseCount * finePulse * 2.0) / 1000000.0;
      
      // Calculate the resistance value using the formula: R = (t / C) / ln(Vc / V)
      resValue = (totalPulseTime / capValue) / log(capVoltage / 5.0);
      
      // Print the results to the serial monitor
      Serial.print("Rough pulse count: ");
      Serial.println(roughPulseCount);
      Serial.print("Fine pulse count: ");
      Serial.println(finePulseCount);
      Serial.print("Capacitor voltage: ");
      Serial.print(capVoltage);
      Serial.println(" V");
      Serial.print("Resistance value: ");
      Serial.print(resValue);
      Serial.println(" ohms");
      
      // Reset the variables and wait for a new measurement
      roughPulseCount = 0;
      finePulseCount = 0;
      capVoltage = 0.0;
      resValue = 0.0;
      fineMode = false;
      delay(5000); // Wait for 5 seconds
      
      // Turn off the LED to indicate ready for a new measurement
      digitalWrite(ledPin, LOW);
    }
    
  } else {
    // Use a rough pulse to charge the capacitor quickly
    pinMode(chargePin, OUTPUT); // Set the charge pin to output mode
    digitalWrite(chargePin, HIGH); // Send a high pulse
    delay(roughPulse); // Wait for a longer time
    pinMode(chargePin, INPUT); // Set the charge pin to input mode
    roughPulseCount++; // Increment the rough pulse count
    
    // Measure the capacitor voltage
    capVoltage = analogRead(measurePin) * (5.0 / 1023.0); // Convert the analog reading to volts
    
    // Check if the capacitor voltage is above the threshold
    if (capVoltage >= threshold) {
      // Switch to the fine mode to refine the measurement
      fineMode = true;
    }
  }
}
