#include <Wire.h>
#include "WalterFeels.h"
void setup() {
  Serial.begin(115200);
  delay(2000);
  WalterFeels::set3v3(true);
  WalterFeels::setI2cBusPower(true);
  delay(200);
  Wire.begin(WFEELS_PIN_I2C_SDA, WFEELS_PIN_I2C_SCL);  // 42, 2
  Serial.println("Scanning...");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf("Found 0x%02X\n", a);
  }
}
void loop(){}