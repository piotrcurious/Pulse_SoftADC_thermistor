// Enhanced backpropagation function for LSTM
void backpropagateLSTM(float input[], float targetResistance) {
  // Declare gradient variables for each gate
  float dWf[1][3] = {{0}};  // Forget gate weight gradient
  float dbf[1] = {0};       // Forget gate bias gradient
  float dWi[1][3] = {{0}};  // Input gate weight gradient
  float dbi[1] = {0};       // Input gate bias gradient
  float dWc[1][3] = {{0}};  // Candidate gate weight gradient
  float dbc[1] = {0};       // Candidate gate bias gradient
  float dWo[1][3] = {{0}};  // Output gate weight gradient
  float dbo[1] = {0};       // Output gate bias gradient
  
  // Compute output error
  float outputError = output[0] - (targetResistance / 1000.0);
  
  // Intermediate gate values from forward pass
  float forgetGate[1], inputGate[1], candidateGate[1], outputGate[1];
  
  // Recompute gate values (this would ideally be stored during forward pass)
  forgetGate[0] = sigmoid(
    Wf[0][0] * input[0] + 
    Wf[0][1] * input[1] + 
    Wf[0][2] * input[2] + 
    bf[0]
  );
  
  inputGate[0] = sigmoid(
    Wi[0][0] * input[0] + 
    Wi[0][1] * input[1] + 
    Wi[0][2] * input[2] + 
    bi[0]
  );
  
  candidateGate[0] = tanh(
    Wc[0][0] * input[0] + 
    Wc[0][1] * input[1] + 
    Wc[0][2] * input[2] + 
    bc[0]
  );
  
  outputGate[0] = sigmoid(
    Wo[0][0] * input[0] + 
    Wo[0][1] * input[1] + 
    Wo[0][2] * input[2] + 
    bo[0]
  );
  
  // Gradient for output gate
  float dOutputGate = outputError * tanh(cellState[0]) * 
                      outputGate[0] * (1 - outputGate[0]);
  
  // Gradient for cell state
  float dCellState = dOutputGate * outputGate[0] * 
                     (1 - tanh(cellState[0]) * tanh(cellState[0])) +
                     outputError * outputGate[0];
  
  // Gradients for input gates
  float dInputGate = dCellState * candidateGate[0] * 
                     inputGate[0] * (1 - inputGate[0]);
  
  float dCandidateGate = dCellState * inputGate[0] * 
                         (1 - candidateGate[0] * candidateGate[0]);
  
  float dForgetGate = dCellState * cellState[0] * 
                      forgetGate[0] * (1 - forgetGate[0]);
  
  // Compute weight gradients
  // Forget gate weights
  dWf[0][0] = dForgetGate * input[0];
  dWf[0][1] = dForgetGate * input[1];
  dWf[0][2] = dForgetGate * input[2];
  dbf[0] = dForgetGate;
  
  // Input gate weights
  dWi[0][0] = dInputGate * input[0];
  dWi[0][1] = dInputGate * input[1];
  dWi[0][2] = dInputGate * input[2];
  dbi[0] = dInputGate;
  
  // Candidate gate weights
  dWc[0][0] = dCandidateGate * input[0];
  dWc[0][1] = dCandidateGate * input[1];
  dWc[0][2] = dCandidateGate * input[2];
  dbc[0] = dCandidateGate;
  
  // Output gate weights
  dWo[0][0] = dOutputGate * input[0];
  dWo[0][1] = dOutputGate * input[1];
  dWo[0][2] = dOutputGate * input[2];
  dbo[0] = dOutputGate;
  
  // Apply weight updates with momentum
  // Forget gate
  for (int j = 0; j < 3; j++) {
    WfMomentum[0][j] = MOMENTUM * WfMomentum[0][j] - LEARNING_RATE * dWf[0][j];
    Wf[0][j] += WfMomentum[0][j];
  }
  bfMomentum[0] = MOMENTUM * bfMomentum[0] - LEARNING_RATE * dbf[0];
  bf[0] += bfMomentum[0];
  
  // Input gate
  for (int j = 0; j < 3; j++) {
    WiMomentum[0][j] = MOMENTUM * WiMomentum[0][j] - LEARNING_RATE * dWi[0][j];
    Wi[0][j] += WiMomentum[0][j];
  }
  biMomentum[0] = MOMENTUM * biMomentum[0] - LEARNING_RATE * dbi[0];
  bi[0] += biMomentum[0];
  
  // Candidate gate
  for (int j = 0; j < 3; j++) {
    WcMomentum[0][j] = MOMENTUM * WcMomentum[0][j] - LEARNING_RATE * dWc[0][j];
    Wc[0][j] += WcMomentum[0][j];
  }
  bcMomentum[0] = MOMENTUM * bcMomentum[0] - LEARNING_RATE * dbc[0];
  bc[0] += bcMomentum[0];
  
  // Output gate
  for (int j = 0; j < 3; j++) {
    WoMomentum[0][j] = MOMENTUM * WoMomentum[0][j] - LEARNING_RATE * dWo[0][j];
    Wo[0][j] += WoMomentum[0][j];
  }
  boMomentum[0] = MOMENTUM * boMomentum[0] - LEARNING_RATE * dbo[0];
  bo[0] += boMomentum[0];
}

// Additional helper function for gradient clipping (optional)
void clipGradients(float* gradients, int size, float threshold) {
  float sumSquared = 0;
  for (int i = 0; i < size; i++) {
    sumSquared += gradients[i] * gradients[i];
  }
  
  float norm = sqrt(sumSquared);
  if (norm > threshold) {
    float scale = threshold / norm;
    for (int i = 0; i < size; i++) {
      gradients[i] *= scale;
    }
  }
}

// Modify the existing sigmoid and tanh functions for numerical stability
float sigmoid(float x) {
  if (x >= 0) {
    float z = exp(-x);
    return 1 / (1 + z);
  } else {
    float z = exp(x);
    return z / (1 + z);
  }
}

float tanh(float x) {
  if (x >= 0) {
    float z = exp(-2 * x);
    return 2 / (1 + z) - 1;
  } else {
    float z = exp(2 * x);
    return z / (1 + z) * 2 - 1;
  }
}
