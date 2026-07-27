#include <WalterModem.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <driver/twai.h>
#include <CANHelper.h>
#include <Wire.h>
//#include <LSM6DSLSensor.h>
#include "WalterFeels.h"


// --- Hardware Wiring ---
#define EX_GNSS_RX_PIN 38
#define EX_GNSS_TX_PIN 39
#define GNSS_BAUD_RATE 9600
#define CAN_TX_PIN GPIO_NUM_15
#define CAN_RX_PIN GPIO_NUM_7

// IMU I2C address (scanner found it at 0x6A)
//#define IMU_I2C_ADDR 0x6A

// ========================================
// Period of messages 1/frequency
// ========================================
#define SPEED_TIME_INTERVAL 20
#define ERROR_TIME_INTERVAL 2000

// ---- GPS speed with smoothing + stationary deadband ----
#define GPS_SPEED_SMOOTH_N   1      // rolling average window
#define GPS_SPEED_DEADBAND   0.0f   // m/s below this reads as 0 (kills jitter)


WalterModem modem;
HardwareSerial GNSSSerial(1);
TinyGPSPlus gps;

// STM IMU object - bound to our Wire instance and the chip's I2C address
//LSM6DSLSensor imu(&Wire, IMU_I2C_ADDR);

unsigned long lastPrintTime = 0;

static float   s_speedBuf[GPS_SPEED_SMOOTH_N] = {0};
static uint8_t s_speedIdx = 0;
static uint8_t s_speedCount = 0;
static float   speed_mps = -1;
static int16_t x_value = -1;          // X accel x100 (m/s^2 * 100) for CAN
static int16_t y_value = -1;
static int16_t z_value = -1;
static long    previousTime = 0;

float getGpsSpeedMps() {
    if (!gps.speed.isValid() || !gps.location.isValid()) {
        s_speedCount = 0;
        s_speedIdx   = 0;
        return -1.0f;
    }

    float raw = gps.speed.mps();          // TinyGPSPlus gives m/s directly

    if (raw < GPS_SPEED_DEADBAND) raw = 0.0f;   // deadband when ~stationary

    // rolling average
    s_speedBuf[s_speedIdx] = raw;
    s_speedIdx = (s_speedIdx + 1) % GPS_SPEED_SMOOTH_N;
    if (s_speedCount < GPS_SPEED_SMOOTH_N) s_speedCount++;

    float sum = 0;
    for (uint8_t i = 0; i < s_speedCount; i++) sum += s_speedBuf[i];
    float avg = sum / s_speedCount;

    // round to nearest 0.1 m/s
    return roundf(avg * 10.0f) / 10.0f;
}

// Read the accelerometer and update x/y/z_value as (m/s^2 * 100).
// The STM library returns acceleration in milli-g (mg). Convert:
//   m/s^2 = mg / 1000 * 9.80665
//   stored value = m/s^2 * 100  ->  mg * 0.980665
// void readImu() {
//     int32_t accel_mg[3] = {0, 0, 0};
//     imu.Get_X_Axes(accel_mg);            // X=accel_mg[0], Y=[1], Z=[2], in mg

//     x_value = (int16_t)lroundf(accel_mg[0] * 0.980665f);
//     y_value = (int16_t)lroundf(accel_mg[1] * 0.980665f);
//     z_value = (int16_t)lroundf(accel_mg[2] * 0.980665f);
// }

void setup() {
  Serial.begin(115200);
  delay(3000); // Give you time to open the Serial Monitor
  Serial.println("\n--- Walter Hardware Local Test ---");
  WalterFeels::set3v3(true);          // power the sensor rail
  WalterFeels::setI2cBusPower(true);  // power the I2C pull-ups
  WalterFeels::setCan(true);
  delay(200);
  Serial.println("3.3v connection open");
  Serial.println("IMU powered on");
  Serial.println("CAN enabled");

  // Wire.begin(WFEELS_PIN_I2C_SDA, WFEELS_PIN_I2C_SCL);

  // // STM library: begin() then enable the accelerometer.
  // // begin() returns LSM6DSL_STATUS_OK (0) on success.
  // if (imu.begin() != LSM6DSL_STATUS_OK) {
  //   Serial.println("ERROR: Failed to start LSM6DSL. Check power pins.");
  //   while (1) { delay(10); } // Halt
  // }
  // if (imu.Enable_X() != LSM6DSL_STATUS_OK) {
  //   Serial.println("ERROR: Failed to enable accelerometer.");
  //   while (1) { delay(10); }
  // }
  // // Optional: gyro too
  // imu.Enable_G();

  // // Set range and data rate (STM API names)
  // imu.Set_X_FS(4);          // +/- 4 g full scale
  // imu.Set_X_ODR(104.0f);    // 104 Hz output data rate

  // Serial.println("SUCCESS: LSM6DSL initialized!");


  // 1. Test Modem Connection (Proves Walter <-> Sequans chip works)
  Serial.print("Initializing LTE Modem... ");
  if (modem.begin(&Serial2)) {          // v1.5.0: pass the modem's UART
      Serial.println("SUCCESS! Modem is responding.");
      modem.definePDPContext(1, "soracom.io");
      // modem.setAuthParams(1, "sora", "sora");   // add if registration fails
      Serial.println("Soracom APN configured. Modem searching for network in background.");
  } else {
      Serial.println("FAILED! Check power, SIM, and ensure the LTE antenna is connected.");
  }

  // 2. Test External GNSS Connection
  Serial.print("Initializing External GNSS on UART... ");
  GNSSSerial.begin(GNSS_BAUD_RATE, SERIAL_8N1, EX_GNSS_RX_PIN, EX_GNSS_TX_PIN);
  Serial.println("STARTED!");
  Serial.println("-----------------------------------");

  CANHelper::setupCAN(CAN_RX_PIN, CAN_TX_PIN);
}

bool speed_message() {
  // Prepare CAN message
  twai_message_t message = { 0 };
  message.identifier = 0x031;    // CAN ID
  message.extd = 0;              // Standard 11-bit ID
  message.data_length_code = 4;  // Data length

  // speed (m/s) x10 as INT16, big-endian in bytes 0-1
  int16_t v = (int16_t)lroundf(speed_mps * 10.0f);
  message.data[0] = (uint8_t)(v >> 8);
  message.data[1] = (uint8_t)(v & 0xFF);

  // X accel (m/s^2 x100) as INT16, big-endian in bytes 2-3
  message.data[2] = (uint8_t)(x_value >> 8);
  message.data[3] = (uint8_t)(x_value & 0xFF);

  return CANHelper::sendMessage(message);
}

void loop() {
  // Constantly read the data stream from the GNSS module
  while (GNSSSerial.available()) {
    gps.encode(GNSSSerial.read());
  }
  speed_mps = getGpsSpeedMps();

  // Read accelerometer (updates x/y/z_value as m/s^2 * 100)
  //readImu();

  unsigned long currentTime = millis();
  if ((currentTime - previousTime) >= SPEED_TIME_INTERVAL) {
    previousTime = currentTime;
    speed_message();
  }

  // Print an update periodically
  if (millis() - lastPrintTime > 150) {
    lastPrintTime = millis();
    Serial.print("[IMU STATUS]");
    Serial.print(" X accel: ");
    Serial.print(x_value / 100.0f);
    Serial.print(" m/s^2| Y accel: ");
    Serial.print(y_value / 100.0f);
    Serial.print(" m/s^2| Z accel: ");
    Serial.print(z_value / 100.0f);
    Serial.print(" [GNSS STATUS] ");

    if (gps.charsProcessed() < 10) {
       Serial.println("ERROR: No data from GNSS module. Check 3.3V, GND, and RX/TX wiring!");
    }
    else if (!gps.location.isValid()) {
       Serial.print("Looking for satellites... (Found: ");
       Serial.print(gps.satellites.value());
       Serial.println(") -> Put the antenna near a window!");
    }
    else {
       Serial.print("FIX ACQUIRED! Lat: ");
       Serial.print(gps.location.lat(), 6);
       Serial.print(" | Lon: ");
       Serial.print(gps.location.lng(), 6);
       Serial.print(" | Sats: ");
       Serial.print(gps.satellites.value());
       Serial.print(" | Speed: ");
       Serial.print(speed_mps * 3.6f);   // m/s -> km/h
       Serial.println(" kmph");
    }
  }
}
