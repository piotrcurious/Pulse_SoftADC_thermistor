// Training hyperparameters
#define LEARNING_RATE 0.01
#define MOMENTUM 0.9
#define MAX_EPOCHS 1000
#define SERIAL_TIMEOUT 5000 // 5 seconds timeout for serial communication

// Training data structures
struct TrainingData {
  float voltage;
  int pulseCount;
  bool flag;
  float targetResistance;
};

// Global arrays to store training data
TrainingData trainingDataset[100]; // Can store up to 100 training samples
int datasetSize = 0;

// Momentum terms for weight updates
float WfMomentum[1][3] = {{0}};
float bfMomentum[1] = {0};
float WiMomentum[1][3] = {{0}};
float biMomentum[1] = {0};
float WcMomentum[1][3] = {{0}};
float bcMomentum[1] = {0};
float WoMomentum[1][3] = {{0}};
float boMomentum[1] = {0};

// Function prototypes for training
void initializeTraining();
bool collectTrainingData();
void trainLSTM();
float computeLoss(float predictedResistance, float targetResistance);
void backpropagateLSTM(float input[], float targetResistance);

// Function to initialize training mode
void initializeTraining() {
  Serial.begin(115200); // Start serial communication
  Serial.setTimeout(SERIAL_TIMEOUT);
  Serial.println("RESISTANCE CALIBRATION MODE ACTIVATED");
  Serial.println("Connect calibration device and send target resistances");
  
  // Reset training dataset
  datasetSize = 0;
  
  // Wait for training data collection
  while (!collectTrainingData()) {
    delay(100);
  }
  
  // Perform LSTM training once data is collected
  trainLSTM();
}

// Function to collect training data via serial port
bool collectTrainingData() {
  if (Serial.available() > 0) {
    // Calibration device sends target resistance
    float targetResistance = Serial.parseFloat();
    
    if (targetResistance > 0) {
      // Measure parameters using original code's methods
      
      // Charge capacitor and count pulses
      int pulseCount = chargeCapacitor();
      
      // Read voltage
      float voltage = readVoltage();
      
      // Check resistance change flag
      bool flag = false;
      if (datasetSize > 0) {
        flag = checkResistanceChange(
          calculateResistance(pulseCount), 
          calculateResistance(trainingDataset[datasetSize-1].pulseCount)
        );
      }
      
      // Store training data
      trainingDataset[datasetSize].voltage = voltage;
      trainingDataset[datasetSize].pulseCount = pulseCount;
      trainingDataset[datasetSize].flag = flag;
      trainingDataset[datasetSize].targetResistance = targetResistance;
      
      datasetSize++;
      
      Serial.print("Collected sample ");
      Serial.print(datasetSize);
      Serial.println(". Send next resistance or 'DONE'");
      
      return (datasetSize >= 100); // Stop if dataset is full
    }
  }
  
  // Check for training completion signal
  if (Serial.available() > 0) {
    String signal = Serial.readStringUntil('\n');
    if (signal == "DONE" && datasetSize > 0) {
      return true; // Training data collection complete
    }
  }
  
  return false;
}

// Function to train LSTM using gradient descent
void trainLSTM() {
  Serial.println("Starting LSTM Training...");
  
  for (int epoch = 0; epoch < MAX_EPOCHS; epoch++) {
    float totalLoss = 0;
    
    // Iterate through training dataset
    for (int i = 0; i < datasetSize; i++) {
      // Prepare input for LSTM
      input[0] = trainingDataset[i].voltage;
      input[1] = trainingDataset[i].pulseCount;
      input[2] = trainingDataset[i].flag;
      
      // Perform forward pass
      lstmForward(input, output, cellState, hiddenState);
      
      // Compute predicted resistance and loss
      float predictedResistance = output[0] * 1000.0; // Scale output to resistance range
      float loss = computeLoss(predictedResistance, trainingDataset[i].targetResistance);
      
      // Accumulate total loss
      totalLoss += loss;
      
      // Perform backpropagation to update weights
      backpropagateLSTM(input, trainingDataset[i].targetResistance);
    }
    
    // Print epoch statistics
    if (epoch % 10 == 0) {
      Serial.print("Epoch ");
      Serial.print(epoch);
      Serial.print(": Average Loss = ");
      Serial.println(totalLoss / datasetSize);
    }
  }
  
  Serial.println("LSTM Training Complete!");
}

// Compute loss between predicted and target resistance
float computeLoss(float predictedResistance, float targetResistance) {
  return 0.5 * pow(predictedResistance - targetResistance, 2);
}

// Simplified backpropagation for LSTM
void backpropagateLSTM(float input[], float targetResistance) {
  // Compute error gradient
  float outputError = output[0] - (targetResistance / 1000.0);
  
  // Simplified weight updates with momentum
  for (int i = 0; i < 1; i++) {
    // Update output gate weights
    for (int j = 0; j < 3; j++) {
      WoMomentum[i][j] = MOMENTUM * WoMomentum[i][j] - LEARNING_RATE * outputError * input[j];
      Wo[i][j] += WoMomentum[i][j];
    }
    boMomentum[i] = MOMENTUM * boMomentum[i] - LEARNING_RATE * outputError;
    bo[i] += boMomentum[i];
  }
}

// Modify setup() to include training mode initialization
void setup() {
  // Check for training mode via a specific pin or serial command
  pinMode(2, INPUT_PULLUP); // Example: Use digital pin 2 to trigger training mode
  if (digitalRead(2) == LOW) {
    initializeTraining();
  }
}

// The rest of the original code (chargeCapacitor, readVoltage, calculateResistance, etc.) 
// remains the same as in the previous implementation
