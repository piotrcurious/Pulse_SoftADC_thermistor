
// Define constants
#define CAPACITOR 10e-6 // Capacitor value in Farads
#define VCC 5.0 // Supply voltage in Volts
#define THRESHOLD 0.63 * VCC // Threshold voltage in Volts
#define CHARGE_PIN 9 // Pin to charge the capacitor
#define INPUT_PIN A10 // Pin to read the capacitor voltage
#define CHANGE_THRESHOLD 1.0 // Resistance change threshold in Ohms

// Define variables
int pulseCount = 0; // Number of pulses to charge or discharge the capacitor
float resistance = 0; // Resistance of the unknown resistor in Ohms
float prevResistance = 0; // Previous resistance of the unknown resistor in Ohms
float voltage = 0; // Voltage across the capacitor in Volts
float error = 0; // Error between measured and predicted resistance in Ohms
bool flag = false; // Flag to indicate resistance change

// Define LSTM parameters
float input[3]; // Input vector for LSTM (voltage, pulse count, and flag)
float output[1]; // Output vector for LSTM
float cellState[1]; // Cell state vector for LSTM
float hiddenState[1]; // Hidden state vector for LSTM

// LSTM weights and biases (randomly initialized, you can train them with your own data)
float Wf[1][3] = {{0.1, 0.2, 0.3}}; // Forget gate weight matrix
float bf[1] = {0.4}; // Forget gate bias vector
float Wi[1][3] = {{0.5, 0.6, 0.7}}; // Input gate weight matrix
float bi[1] = {0.8}; // Input gate bias vector
float Wc[1][3] = {{0.9, 1.0, 1.1}}; // Candidate gate weight matrix
float bc[1] = {1.2}; // Candidate gate bias vector
float Wo[1][3] = {{1.3, 1.4, 1.5}}; // Output gate weight matrix
float bo[1] = {1.6}; // Output gate bias vector

// LSTM activation functions
float sigmoid(float x) {
  return 1 / (1 + exp(-x));
}

float tanh(float x) {
  return (exp(x) - exp(-x)) / (exp(x) + exp(-x));
}

// LSTM forward pass function
void lstmForward(float input[], float output[], float cellState[], float hiddenState[]) {
  float forgetGate[1]; // Forget gate output vector
  float inputGate[1]; // Input gate output vector
  float candidateGate[1]; // Candidate gate output vector
  float outputGate[1]; // Output gate output vector
  
  // Compute forget gate output
  for (int i = 0; i < 1; i++) {
    forgetGate[i] = sigmoid(Wf[i][0] * input[0] + Wf[i][1] * input[1] + Wf[i][2] * input[2] + bf[i]);
  }
  
  // Compute input gate output
  for (int i = 0; i < 1; i++) {
    inputGate[i] = sigmoid(Wi[i][0] * input[0] + Wi[i][1] * input[1] + Wi[i][2] * input[2] + bi[i]);
  }
  
  // Compute candidate gate output
  for (int i = 0; i <1; i++) {
    candidateGate[i] = tanh(Wc[i][0] * input[0] + Wc[i][1] * input[1] + Wc[i][2] * input[2] + bc[i]);
  }
  
  // Compute output gate output
  for (int i = 0; i < 1; i++) {
    outputGate[i] = sigmoid(Wo[i][0] * input[0] + Wo[i][1] * input[1] + Wo[i][2] * input[2] + bo[i]);
  }
  
  // Update cell state and hidden state
  for (int i = 0; i < 1; i++) {
    cellState[i] = forgetGate[i] * cellState[i] + inputGate[i] * candidateGate[i];
    hiddenState[i] = outputGate[i] * tanh(cellState[i]);
    output[i] = hiddenState[i];
  }
}

void setup() {
  
}

void loop() {
  
   pulseCount = chargeCapacitor(); // Charge the capacitor and count the pulses
   
   resistance = calculateResistance(pulseCount); // Calculate the resistance from the pulse count
   
   voltage = readVoltage(); // Read the voltage across the capacitor
   
   flag = checkResistanceChange(resistance, prevResistance); // Check if the resistance did change and set the flag
   
   error = predictError(voltage, pulseCount, flag); // Predict the error from the voltage, pulse count, and flag using LSTM
   
   resistance += error; // Correct the resistance with the error
   
   Serial.print("Resistance: "); 
   Serial.print(resistance); 
   Serial.println(" Ohms"); 
   
   prevResistance = resistance; // Update the previous resistance value
   
   pulseCount = dischargeCapacitor(); // Discharge the capacitor and count the pulses
   
   resistance = calculateResistance(pulseCount); // Recalculate the resistance from the pulse count
   
   voltage = readVoltage(); // Read the voltage across the capacitor
   
   flag = checkResistanceChange(resistance, prevResistance); // Check if the resistance did change and set the flag
   
   error = predictError(voltage, pulseCount, flag); // Repredict the error from the voltage, pulse count, and flag using LSTM
   
   resistance += error; // Recorrect the resistance with the error
   
   Serial.print("Resistance: "); 
   Serial.print(resistance); 
   Serial.println(" Ohms"); 
   
   prevResistance = resistance; // Update the previous resistance value
   
   delay(1000); // Wait for a second
}

// Function to charge the capacitor and count the pulses
int chargeCapacitor() {
  int count = 0; // Initialize the pulse count
  pinMode(CHARGE_PIN, OUTPUT); // Set the charge pin as output
  digitalWrite(CHARGE_PIN, HIGH); // Start charging the capacitor
  while (voltage < THRESHOLD) { // While the voltage is below the threshold
    count++; // Increment the pulse count
    delayMicroseconds(10); // Wait for 10 microseconds
    pinMode(CHARGE_PIN, INPUT); // Set the charge pin as input to end the pulse
    voltage = readVoltage(); // Read the voltage across the capacitor
    pinMode(CHARGE_PIN, OUTPUT); // Set the charge pin as output to start the next pulse
    digitalWrite(CHARGE_PIN, HIGH); // Continue charging the capacitor
  }
  digitalWrite(CHARGE_PIN, LOW); // Stop charging the capacitor
  return count; // Return the pulse count
}

// Function to calculate the resistance from the pulse count
float calculateResistance(int count) {
  float time = count * 10e-6; // Calculate the time in seconds
  float r = time / (CAPACITOR * log(1 / (1 - THRESHOLD / VCC))); // Calculate the resistance in Ohms
  return r; // Return the resistance
}

// Function to read the voltage across the capacitor
float readVoltage() {
  int value = analogRead(INPUT_PIN); // Read the analog value from 0 to 1023
  float v = value * VCC / 1024.0; // Convert the value to voltage in Volts
  return v; // Return the voltage
}

// Function to check if the resistance did change and set the flag
bool checkResistanceChange(float r1, float r2) {
  bool f = false; // Initialize the flag to false
  if (abs(r1 - r2) > CHANGE_THRESHOLD) { // If the difference between the current and previous resistance is larger than the threshold
    f = true; // Set the flag to true
  }
  return f; // Return the flag
}

// Function to predict the error from the voltage, pulse count, and flag using LSTM
float predictError(float v, int c, bool f) {
  input[0] = v; // Set the input vector to the voltage value
  input[1] = c; // Set the input vector to the pulse count value
  input[2] = f; // Set the input vector to the flag value
  lstmForward(input, output, cellState, hiddenState); // Perform a forward pass of LSTM
  float e = output[0]; // Get the output vector as the error value
  return e; // Return the error
}

// Function to discharge the capacitor and count the pulses
int dischargeCapacitor() {
  int count = 0; // Initialize the pulse count
  pinMode(CHARGE_PIN, OUTPUT); // Set the charge pin as output
  digitalWrite(CHARGE_PIN, LOW); // Start discharging the capacitor
  while (voltage > THRESHOLD) { // While the voltage is above the threshold
    count--; // Decrement the pulse count
    delayMicroseconds(10); // Wait for 10 microseconds
    pinMode(CHARGE_PIN, INPUT); // Set the charge pin as input to end the pulse
    voltage = readVoltage(); // Read the voltage across the capacitor
    pinMode(CHARGE_PIN, OUTPUT); // Set the charge pin as output to start the next pulse
    digitalWrite(CHARGE_PIN, LOW); // Continue discharging the capacitor
  }
  digitalWrite(CHARGE_PIN, HIGH); // Stop discharging the capacitor
  return count; // Return the pulse count
}
