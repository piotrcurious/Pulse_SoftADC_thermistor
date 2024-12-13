
// SoftADC thermistor library for Arduino Uno
// by Bing

// Define the maximum number of channels
#define MAX_CHANNELS 8

// Define the charge and discharge thresholds
#define CHARGE_THRESHOLD 900
#define DISCHARGE_THRESHOLD 100

// Define the default pulse length in microseconds
#define DEFAULT_PULSE_LENGTH 10

// Define a struct to store the channel data
typedef struct {
  uint8_t pin; // the pin number
  uint32_t chargeCount; // the charge pulse count
  uint32_t dischargeCount; // the discharge pulse count
  uint32_t chargeAverage; // the charge pulse average
  uint32_t dischargeAverage; // the discharge pulse average
  bool charging; // the charging state
  bool active; // the active state
} Channel;

// Declare an array of channels
Channel channels[MAX_CHANNELS];

// Declare a variable to store the pulse length
uint16_t pulseLength;

// Initialize the library with the given pins and pulse length
void SoftADC_begin(uint8_t pins[], uint8_t numPins, uint16_t pulseLen = DEFAULT_PULSE_LENGTH) {
  // Check if the number of pins is valid
  if (numPins > MAX_CHANNELS) {
    numPins = MAX_CHANNELS;
  }
  
  // Initialize the pulse length
  pulseLength = pulseLen;
  
  // Initialize the channels
  for (uint8_t i = 0; i < numPins; i++) {
    // Store the pin number
    channels[i].pin = pins[i];
    
    // Set the pin to input mode and disable pullups
    pinMode(channels[i].pin, INPUT);
    digitalWrite(channels[i].pin, LOW);
    
    // Reset the pulse counts and averages and charging state
    channels[i].chargeCount = 0;
    channels[i].dischargeCount = 0;
    channels[i].chargeAverage = 0;
    channels[i].dischargeAverage = 0;
    channels[i].charging = false;
    
    // Set the channel to active
    channels[i].active = true;
  }
}

// Update the library and perform measurements
void SoftADC_update() {
  // Loop through all the channels
  for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
    // Check if the channel is active
    if (channels[i].active) {
      // Check if the channel is charging or discharging
      if (channels[i].charging) {
        // Set the pin to output mode and write high
        pinMode(channels[i].pin, OUTPUT);
        digitalWrite(channels[i].pin, HIGH);
        
        // Increment the charge pulse count
        channels[i].chargeCount++;
        
        // Delay for the pulse length
        delayMicroseconds(pulseLength);
        
        // Set the pin to input mode and read the analog value
        pinMode(channels[i].pin, INPUT);
        int value = analogRead(channels[i].pin);
        
        // Check if the value is above the charge threshold
        if (value >= CHARGE_THRESHOLD) {
          // Stop charging and start discharging
          channels[i].charging = false;
          
          // Update the charge pulse average using a simple moving average formula
          channels[i].chargeAverage = (channels[i].chargeAverage + channels[i].chargeCount) / 2;
          
          // Reset the charge pulse count
          channels[i].chargeCount = 0;
        }
      } else {
        // Set the pin to output mode and write low
        pinMode(channels[i].pin, OUTPUT);
        digitalWrite(channels[i].pin, LOW);
        
        // Increment the discharge pulse count
        channels[i].dischargeCount++;
        
        // Delay for the pulse length
        delayMicroseconds(pulseLength);
        
        // Set the pin to input mode and read the analog value
        pinMode(channels[i].pin, INPUT);
        int value = analogRead(channels[i].pin);
        
        // Check if the value is below the discharge threshold
        if (value <= DISCHARGE_THRESHOLD) {
          // Stop discharging and start charging
          channels[i].charging = true;
          
          // Update the discharge pulse average using a simple moving average formula
          channels[i].dischargeAverage = (channels[i].dischargeAverage + channels[i].dischargeCount) / 2;
          
          // Reset the discharge pulse count
          channels[i].dischargeCount = 0;
        }
      }
    }
  }
}

// Get the measurement of a channel in pulses
uint32_t SoftADC_read(uint8_t channel) {
  // Check if the channel is valid and active
  if (channel < MAX_CHANNELS && channels[channel].active) {
    // Return the average of charge and discharge pulse averages of the channel
    return (channels[channel].chargeAverage + channels[channel].dischargeAverage) / 2;
  } else {
    // Return zero as an error value
    return 0;
  }
}

// Set a channel to inactive or active
void SoftADC_setActive(uint8_t channel, bool state) {
  // Check if the channel is valid
  if (channel < MAX_CHANNELS) {
    // Set the active state of the channel
    channels[channel].active = state;
    
    // Reset the pulse counts and averages and charging state of the channel
    channels[channel].chargeCount = 0;
    channels[channel].dischargeCount = 0;
    channels[channel].chargeAverage = 0;
    channels[channel].dischargeAverage = 0;
    channels[channel].charging = false;
    
    // Set the pin to input mode and disable pullups
    pinMode(channels[channel].pin, INPUT);
    digitalWrite(channels[channel].pin, LOW);
  }
}

// Calculate the resistance of the thermistor from the pulse average using a linear approximation formula
// R = k * N * T + b, where k and b are constants depending on the thermistor and capacitor values, and T is the pulse length in seconds
float SoftADC_calculateResistance(uint8_t channel) {
  // Check if the channel is valid and active
  if (channel < MAX_CHANNELS && channels[channel].active) {
    // Define the constants k and b (change them according to your thermistor and capacitor values)
    const float k = 0.01; // ohms per pulse per second
    const float b = 100.0; // ohms
    
    // Get the pulse average of the channel
    uint32_t N = SoftADC_read(channel);
    
    // Convert the pulse length to seconds
    float T = pulseLength / 1000000.0;
    
    // Calculate and return the resistance of the thermistor
    return k * N * T + b;
  } else {
    // Return zero as an error value
    return 0.0;
  }
}


