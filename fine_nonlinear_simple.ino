
// Define the pins
const int chargePin = 2; // The pin that charges the capacitor
const int sensePin = A0; // The pin that measures the voltage across the capacitor
const int ledPin = 13; // The pin that controls the LED

// Define the constants
const float capacitance = 0.0001; // The capacitance of the capacitor in farads
const float threshold = 4.0; // The voltage threshold in volts
const int pulseWidth = 10; // The width of the charge pulse in milliseconds

// Define the variables
int pulseCount = 0; // The number of pulses needed to charge the capacitor above the threshold
float resistance = 0.0; // The resistance of the unknown resistor in ohms
float voltage = 0.0; // The voltage across the capacitor in volts

void setup() {
  // Initialize the serial monitor
  Serial.begin(9600);
  
  // Set the charge pin as output and low
  pinMode(chargePin, OUTPUT);
  digitalWrite(chargePin, LOW);
  
  // Set the LED pin as output and low
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
}

void loop() {
  // Charge the capacitor with a pulse
  digitalWrite(chargePin, HIGH); // Start the pulse
  delay(pulseWidth); // Wait for the pulse width
  pinMode(chargePin, INPUT); // End the pulse by setting the pin as input
  
  // Increment the pulse count
  pulseCount++;
  
  // Read the voltage across the capacitor
  voltage = analogRead(sensePin) * (5.0 / 1023.0); // Convert from analog value to volts
  
  // Check if the voltage is above the threshold
  if (voltage >= threshold) {
    // Turn on the LED to indicate that the capacitor is charged
    digitalWrite(ledPin, HIGH);
    
    // Calculate the resistance using the formula R = (t / C) / ln(V / (V - V0))
    // where t is the total time, C is the capacitance, V is the final voltage,
    // V0 is the initial voltage (assumed to be zero), and ln is the natural logarithm
    resistance = (pulseCount * pulseWidth / 1000.0) / capacitance / log(voltage / (5.0 - voltage));
    
    // Print the results to the serial monitor
    Serial.print("Pulse count: ");
    Serial.println(pulseCount);
    Serial.print("Voltage: ");
    Serial.print(voltage);
    Serial.println(" V");
    Serial.print("Resistance: ");
    Serial.print(resistance);
    Serial.println(" ohms");
    
    // Reset the pulse count and voltage
    pulseCount = 0;
    voltage = 0.0;
    
    // Wait for a second before discharging the capacitor
    delay(1000);
    
    // Discharge the capacitor by setting the charge pin as output and low
    pinMode(chargePin, OUTPUT);
    digitalWrite(chargePin, LOW);
    
    // Turn off the LED to indicate that the capacitor is discharged
    digitalWrite(ledPin, LOW);
    
    // Wait for a second before repeating the loop
    delay(1000);
  }
}
