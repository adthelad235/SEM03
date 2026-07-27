// Need to add a check to shut off motors if no readings recieved in x milliseconds
// Need to check the steering angle reading is in the correct range
// NEed to check the potentiometer reading is in the correct range


/*
  E-diff Control Logic

  This code is intended to verify the ediff control logic on Arduino Mega
  before ESP32 + CAN bus implementation.

  INPUTS
  1. Throttle input from pedal potentiometer
  2. Steering input from hall-effect steering angle sensor
  3. Direction input (forward / reverse / neutral)

  OUTPUTS
  1. leftOutput  -> LEFT wheel command
  2. rightOutput -> RIGHT wheel command

  CONTROL IDEA
  - Throttle sets the speed demand
  - Steering angle sets how that speed demand is split between the left and right wheels
  - Equal wheel outputs = straight driving
  - Unequal wheel outputs = turning by e-diff

  NOTE
  - This version is for Arduino Mega testing, so ADC_MAX = 1023
  - When moved to ESP32 later, ADC_MAX can be changed to 4095
*/
#include "config.h"
#include <driver/twai.h>
#include <VescUart.h>
#include <CANHelper.h>

// VESC
VescUart UART;
VescUart UART2;

// Global Variables for current states
int rawThrottle = 0;
bool throttleRead = false;
bool angleRead = false;
float steeringAngleDegree = 0.0f;
unsigned long messageCount = 0;
DirectionState currentDirection = NEUTRAL;


// FUNCTION TO LIMIT VALUES TO SAFE RANGE
float clampValue(float x, float minVal, float maxVal) {
  if (x < minVal) return minVal;
  if (x > maxVal) return maxVal;
  return x;
}

// FUNCTION TO READ AND NORMALISE THROTTLE
// Converts pedal potentiometer reading into a value from 0.0 (no throttle) to 1.0 (full throttle)
float getThrottleDemand(int rawThrottle) {
  return clampValue(rawThrottle / (float)(ADC_MAX - ADC_MIN), 0.0f, 1.0f);
}

// DIRECTION TEXT FOR SERIAL MONITOR
const char* directionText(DirectionState dir) {
  if (dir == FORWARD) return "FORWARD";
  if (dir == REVERSE) return "REVERSE";
  return "NEUTRAL";
}

void decodeThrottleMessage(twai_message_t message){
  rawThrottle = (int)((message.data[0] << 8) | message.data[1]);
  uint8_t gear = message.data[2];
  if (gear == 0){
    currentDirection = NEUTRAL;
  }
  if (gear == 1){
    currentDirection = FORWARD;
  }
  if (gear == 2){
    currentDirection = REVERSE;
  }
}

void decodeSteeringAngleMessage(twai_message_t message){
  steeringAngleDegree = (int)((message.data[0] << 8) | message.data[1])/100;
}

void VESCSetup(){
  Serial.println("Starting E-diff Test");
}

void VESCLoop(int throttleRaw, DirectionState direction, float steeringAngleDeg){

  // 1. PROCESS THROTTLE INPUT
  float throttleDemand = getThrottleDemand(throttleRaw);
  float baseOutput = throttleDemand * MAX_OUTPUT;

  // 2. NORMALISE STEERING DEMAND
  // -1.0 = full left, 0.0 = straight, +1.0 = full right
  float steeringDemand = clampValue(steeringAngleDeg / MAX_STEER_ANGLE_DEG, -1.0f, 1.0f);

  // 4. PERFORM E-DIFF SPLIT
  // Positive steering demand = right turn request:
  // left wheel output increases, right wheel output decreases
  float steerBias = STEER_GAIN * steeringDemand * baseOutput;

  float leftOutput = baseOutput + steerBias;
  float rightOutput = baseOutput - steerBias;

  // Limit outputs so they remain within allowed range
  leftOutput = clampValue(leftOutput, -MAX_OUTPUT, MAX_OUTPUT);
  rightOutput = clampValue(rightOutput, -MAX_OUTPUT, MAX_OUTPUT);

  // 5. APPLY DIRECTION LOGIC
  if (direction == REVERSE) {
    leftOutput = -leftOutput;
    rightOutput = -rightOutput;
  } else if (direction == NEUTRAL) {
    leftOutput = 0.0f;
    rightOutput = 0.0f;
  }

  // 6. PRINT DATA ON SERIAL MONITOR
  Serial.print("ThrottleRaw: ");
  Serial.print(throttleRaw);
  Serial.print(" | SteeringAngleDeg: ");
  Serial.print(steeringAngleDeg, 2);
  Serial.print(" | Direction: ");
  Serial.print(directionText(direction));
  Serial.print(" | LeftWheel: ");
  Serial.print(leftOutput, 3);
  Serial.print(" | RightWheel: ");
  Serial.println(rightOutput, 3);

  // 7. SET VESC
  
  UART.setRPM((int)rightOutput);
  UART2.setRPM((int)leftOutput);

}

void CANReceiveLoop(){
  twai_message_t message;
  
  // Try to receive a message (wait up to 100ms)
  esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(100));
  
  if (result == ESP_OK) {
    messageCount++;
    // Print message details
    CANHelper::printCANMessage(message, messageCount);

    if (message.identifier == 0x021){
      decodeThrottleMessage(message);
      throttleRead = true;
    }
    else if (message.identifier == 0x011){
      decodeSteeringAngleMessage(message);
      angleRead = true;
    }
  }
  
  uint32_t alerts;
  twai_read_alerts(&alerts, pdMS_TO_TICKS(0));
  if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
    Serial.println("Warning: RX Queue Full!");
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, VESC1_RX_PIN, VESC1_TX_PIN);
  delay(1000);

  pinMode(0, INPUT_PULLUP);
  delay(100);

  UART.setSerialPort(&Serial1);

  CANHelper::setupCAN(CAN_RX_PIN, CAN_TX_PIN);

  VESCSetup();
}

void loop() {
  // put your main code here, to run repeatedly:
  while(!(throttleRead && angleRead)){
    CANReceiveLoop();
  }

  VESCLoop(rawThrottle, currentDirection, steeringAngleDegree);

  throttleRead = false;
  angleRead = false;
}
