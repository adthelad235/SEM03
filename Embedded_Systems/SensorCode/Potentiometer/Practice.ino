#include <LiquidCrystal.h>

const int POTENTIOMETER_PIN = A5;
const int RS = 7, EN = 8, D4 = 9, D5 = 10, D6 = 11, D7 = 12;
const LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.setCursor(0, 1);
  lcd.print("Hello");
}

void loop() {
  //put your main code here, to run repeatedly:
  int analogValue = analogRead(POTENTIOMETER_PIN);
  lcd.setCursor(0, 1);
  lcd.print(analogValue);
  Serial.println(analogValue);
}
// const int RED_LED_PIN = 9;
// const int YELLOW_LED_PIN = 6;
// const int GREEN_LED_PIN = 3;
// const int RED_THRESHOLD = 333;
// const int YELLOW_THRESHOLD = 666;

  // pinMode(RED_LED_PIN, OUTPUT);
  // pinMode(YELLOW_LED_PIN, OUTPUT);
  // pinMode(GREEN_LED_PIN, OUTPUT);

// if(analogValue < RED_THRESHOLD){
  //   Serial.println("RED");
  //   digitalWrite(RED_LED_PIN, HIGH);
  //   digitalWrite(YELLOW_LED_PIN, LOW);
  //   digitalWrite(GREEN_LED_PIN, LOW);
  // }
  // else if(analogValue < YELLOW_THRESHOLD){
  //   Serial.println("YELLOW");
  //   digitalWrite(YELLOW_LED_PIN, HIGH);
  //   digitalWrite(GREEN_LED_PIN, LOW);
  //   digitalWrite(RED_LED_PIN, LOW);
  // }
  // else{
  //   Serial.println("GREEN");
  //   digitalWrite(GREEN_LED_PIN, HIGH);
  //   digitalWrite(YELLOW_LED_PIN, LOW);
  //   digitalWrite(RED_LED_PIN, LOW);
  // }
