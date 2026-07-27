#include "config.h"
#include "hall_position.h"
#include <CANHelper.h>

HallPositionSensor sensor;

unsigned long previousTime = 0;
int errorLastSent[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

void setup() {
  Serial.begin(115200);
  delay(500);

  setup_sensors();

  CANHelper::setupCAN(CAN_RX_PIN, CAN_TX_PIN);

}

void loop() {
  PositionResult result = sensor.update();
  print_sensor_output(result);
  int error = result.status;

  unsigned long currentTime = millis();
  if (error < 2 && (currentTime - previousTime) >= ANGLE_TIME_INTERVAL) {
    previousTime = currentTime;
    if(!angle_message(result)){
      error = 7;
    }
  }

  if (error > 1 && (currentTime - errorLastSent[error]) >= ERROR_TIME_INTERV AL) {
    error_message(result);
    errorLastSent[error] = currentTime;
  }
  delay(1000);
}

void setup_sensors(){
  Serial.println("Hall Effect Linear Position Sensor");
  Serial.println("-----------------------------------");

  sensor.begin();

  Serial.println("Calibrating baselines — keep magnet away from array...");
  delay(1000);
  sensor.calibrate();
  Serial.println("Calibration complete.\n");

  // Print sensor health after calibration
  Serial.println("Sensor baseline values:");
  for (int i = 0; i < N_SENSORS; i++) {
    Serial.printf("  Sensor %d (mux ch %d): baseline = %d\n",
                  i, SENSOR_MUX_CH[i], sensor.getBaseline(i));
  }
  Serial.println();
}

void print_sensor_output(PositionResult result){
    // --- Print position or error ---
  switch (result.status) {
    case STATUS_OK:
      Serial.printf("Position: %6.2f mm  [OK, %d/%d sensors]\n",
                    result.position_mm, result.valid_count, N_SENSORS);
      break;

    case STATUS_OK_DEGRADED:
      Serial.printf("Position: %6.2f mm  [DEGRADED — %d/%d sensors active]\n",
                    result.position_mm, result.valid_count, N_SENSORS);
      break;

    case ERROR_NO_MAGNET:
      Serial.println("ERROR: No magnet detected (signal below threshold)");
      break;

    case ERROR_COVERAGE:
      Serial.printf("ERROR: Insufficient sensor coverage (%d valid sensors, need %d)\n",
                    result.valid_count, MIN_VALID_SENSORS);
      break;

    case ERROR_SIGNAL_ANOMALY:
      Serial.println("ERROR: Signal anomaly (bimodal or implausible shape)");
      break;

    case ERROR_OUT_OF_RANGE:
      Serial.println("ERROR: Calculated position out of array bounds");
      break;

    case ERROR_ALL_DEAD:
      Serial.println("ERROR: All sensors dead — check wiring");
      break;
  }

  // Print individual sensor health if any are faulty
  if (result.sensor_health != 0xFF >> (8 - N_SENSORS)) {  // not all valid
    Serial.print("  Sensor health: ");
    for (int i = 0; i < N_SENSORS; i++) {
      bool valid = (result.sensor_health >> i) & 1;
      Serial.printf("S%d:%s ", i, valid ? "OK" : "FAULT");
    }
    Serial.println();
  }
}

bool angle_message(PositionResult result){
  // Calculate steering angle
  float angle = calculate_steering_angle(result.position_mm);
  if (fabsf(angle) > MAX_STEERING_ANGLE){
    return false;
  }
  int16_t adjustedAngle = angle * 100;
    
  // Prepare CAN message
  twai_message_t message = { 0 };
  message.identifier = 0x011;    // CAN ID
  message.extd = 0;              // Standard 11-bit ID
  message.data_length_code = 2;  // Data length

  // Pack data into CAN message
  message.data[0] = (adjustedAngle >> 8) & 0xFF;  // High byte
  message.data[1] = adjustedAngle & 0xFF;         // Low byte

  // Transmit message
  return CANHelper::sendMessage(message);
}

bool error_message(PositionResult result){
  // Prepare CAN message
  twai_message_t message = { 0 };
  message.identifier = 0x010;    // CAN ID
  message.extd = 0;              // Standard 11-bit ID
  message.data_length_code = 3;  // Data length

  // Pack data into CAN message
  message.data[0] = result.status;
  message.data[1] = (result.sensor_health >> 8) & 0xFF;  // High byte
  message.data[2] = result.sensor_health & 0xFF;         // Low byte

  // Transmit message
  return  CANHelper::sendMessage(message);
}

float calculate_steering_angle(float position){
  return (position - CENTRE_POSITION) * MAX_STEERING_ANGLE / MAX_DISTANCE_MOVED;
}


