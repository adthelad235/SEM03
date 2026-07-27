// Define pins connected to the multiplexer
#define S0 18
#define S1 19
#define S2 22
#define S3 23
#define MUX_ENABLE_PIN 4  // A GPIO pin to control the ENABLE pin
#define MUX_SIGNAL_PIN 34 // The ESP32 ADC pin connected to the MUX's COM pin

// Store the control pins in an array 
const uint8_t controlPins[] = {S0, S1, S2, S3};

const int numSensors = 9;
int sensorValues[numSensors];

// A lookup table to easily set the correct address for channels 0-15
const uint8_t muxAddress[16][4] = {
  {0,0,0,0}, // channel 0
  {1,0,0,0}, // channel 1
  {0,1,0,0}, // channel 2
  {1,1,0,0}, // channel 3
  {0,0,1,0}, // channel 4
  {1,0,1,0}, // channel 5
  {0,1,1,0}, // channel 6
  {1,1,1,0}, // channel 7
  {0,0,0,1}, // channel 8
  {1,0,0,1}, // channel 9
  {0,1,0,1}, // channel 10
  {1,1,0,1}, // channel 11
  {0,0,1,1}, // channel 12
  {1,0,1,1}, // channel 13
  {0,1,1,1}, // channel 14
  {1,1,1,1}  // channel 15
};

void setup() {
  Serial.begin(115200);

  // Set up the control pins as outputs
  for (int i = 0; i < 4; i++) {
    pinMode(controlPins[i], OUTPUT);
    // Set them to LOW
    digitalWrite(controlPins[i], LOW);
  }

  // Set up the ENABLE pin
  pinMode(MUX_ENABLE_PIN, OUTPUT);
  // Set the enable pin to low to activate the multiplexer
  digitalWrite(MUX_ENABLE_PIN, LOW);

  // The signal pin is an input
  pinMode(MUX_SIGNAL_PIN, INPUT);

  Serial.println("Setup complete.");
}

// This function selects a channel on the multiplexer
void selectMuxChannel(int channel) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(controlPins[i], muxAddress[channel][i]);
  }
  // A short delay might be needed for the signal to stabilize after switching
  delayMicroseconds(10);
}

void loop() {
  // Loop through all 9 sensor channels
  for (int channel = 0; channel < numSensors; channel++) {
    selectMuxChannel(channel);
    // Read the analog value from the multiplexer's common pin
    sensorValues[channel] = analogRead(MUX_SIGNAL_PIN);
  }

  // The array sensorValues now holds readings from each of the nine inputs to the multiplexers

  // Print the readings to the Serial Monitor
  Serial.print("Readings: ");
  for (int i = 0; i < numSensors; i++) {
    Serial.print(sensorValues[i]);
    if (i < numSensors - 1) {
      Serial.print(", ");
    }
  }
  Serial.println();

  // Wait before next loop
  delay(100);
}