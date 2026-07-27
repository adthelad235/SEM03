#include <driver/twai.h>
#include <CANHelper.h>
#include "config.h"
#include "AcceleratorPedal.h"

//Tested with the USB-CAN-A monitor and a single potentiometer and all seems to be working well even at 50Hz
//Need to test with the dual potentiometer and the gear switch but code all looks like it is working well
//More testing on the error messages can be done and the dual potentiometer needs configured depending on what value we want to send

unsigned long previousTime = 0;
int errorLastSent[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(GEAR1_PIN, INPUT_PULLDOWN);
  pinMode(GEAR2_PIN, INPUT_PULLDOWN);

  CANHelper::setupCAN(CAN_RX_PIN, CAN_TX_PIN);
}

void loop() {
  unsigned long currentTime = millis();
  if ((currentTime - previousTime) >= ACCELERATOR_TIME_INTERVAL) {
    previousTime = currentTime;
    accelerator_message();
  }

  uint8_t error = AcceleratorPedal::getError();
  if (error != 0 && (currentTime - errorLastSent[error]) >= ERROR_TIME_INTERVAL) {
    errorLastSent[error] = currentTime;
    error_message(error);
    AcceleratorPedal::resetError();
  }
}

bool accelerator_message() {
  // Read potentiometers (0-4095 for ESP32 ADC)
  uint16_t pot1 = analogRead(POT1_PIN);
  uint16_t pot2 = analogRead(POT2_PIN);

  // Read gear switches
  bool gear1 = digitalRead(GEAR1_PIN);
  bool gear2 = digitalRead(GEAR2_PIN);

  // Process accelerator pedal values
  uint16_t acceleratorValue = AcceleratorPedal::potentiometerReading(pot1, pot2);
  uint8_t gear = AcceleratorPedal::gearPosition(gear1, gear2);

  // Prepare CAN message
  twai_message_t message = { 0 };
  message.identifier = 0x021;    // CAN ID
  message.extd = 0;              // Standard 11-bit ID
  message.data_length_code = 3;  // Data length

  // Pack data into CAN message
  message.data[0] = (acceleratorValue >> 8) & 0xFF;  // High byte
  message.data[1] = acceleratorValue & 0xFF;         // Low byte
  message.data[2] = gear;

  // Transmit message
  return  CANHelper::sendMessage(message);
}

bool error_message(uint8_t error) {

  // Prepare CAN message
  twai_message_t message = { 0 };
  message.identifier = 0x020;    // CAN ID
  message.extd = 0;              // Standard 11-bit ID
  message.data_length_code = 1;  // Data length

  // Pack data into CAN message
  message.data[0] = error;

  // Transmit message
  return  CANHelper::sendMessage(message);
}