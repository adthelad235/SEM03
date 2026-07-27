// Libraries
#define _USE_MATH_DEFINES
#include <math.h>
#include<time.h>
#include<LiquidCrystal.h>

// Constants
const float RADIUS = 100; // Radius of wheel in m
const float CIRCUMFERENCE = 2 * M_PI * RADIUS; // Circumference of the wheel in m
const int HALL_PIN = 3;
const int RS = 7, EN = 8, D4 = 9, D5 = 10, D6 = 11, D7 = 12;
const LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);


// Variables
volatile bool currentState = LOW;
volatile bool prevState = LOW;
volatile float currentSpeed = 0;
volatile long long prevTime = millis();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(HALL_PIN, INPUT);
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Speed:");
  lcd.setCursor(11, 1);
  lcd.print("km/h");
}

void loop() {
  //put your main code here, to run repeatedly:
  currentState = digitalRead(HALL_PIN);
  if(currentState == true && prevState == false){
    long long time = millis();
    currentSpeed = calculateSpeed(time - prevTime);
    prevTime = time;
    lcd.setCursor(0, 1);
    lcd.print(currentSpeed);
  }
  prevState = currentState;

}

float calculateSpeed(float timeGap){
  return convert(CIRCUMFERENCE/timeGap);
}

float convert(float value){
  return value * 3.6;
} 