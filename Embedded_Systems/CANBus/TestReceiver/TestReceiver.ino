//Tested and works correctly can be used for testing messages between esps


#include <driver/twai.h>
#include <LiquidCrystal.h>

// Pin definitions
#define CAN_TX_PIN GPIO_NUM_21
#define CAN_RX_PIN GPIO_NUM_19

// Message counter
unsigned long messageCount = 0;
unsigned long lastErrorTime = 0;
bool showingError = false;

LiquidCrystal lcd(21, 22, 18, 19, 23, 27);

void setup() {
  Serial.begin(115200);
  delay(1000);
  lcd.begin(16, 2);
  
  Serial.println("ESP32 CAN Receiver (TWAI) Starting...");
  Serial.println("Waiting for CAN messages...");
  Serial.println("--------------------------------------------------------");
  
  // Configure CAN parameters
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_PIN, 
    (gpio_num_t)CAN_RX_PIN, 
    TWAI_MODE_NORMAL
  );
  
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("TWAI driver installed");
  } else {
    Serial.println("Failed to install TWAI driver");
    return;
  }
  
  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("TWAI started");
  } else {
    Serial.println("Failed to start TWAI");
    return;
  }
  
  // Reconfigure alerts to detect reception (optional)
  twai_reconfigure_alerts(TWAI_ALERT_RX_DATA, NULL);
}

void loop() {
  twai_message_t message;
  
  // Try to receive a message (wait up to 100ms)
  esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(100));
  
  if (result == ESP_OK) {
    messageCount++;
    
    // Print message details
    printCANMessage(message);
    
    // Optional: Decode based on message ID
    //decodeTWAI(message);
    if(message.identifier == 0x20){
      lastErrorTime = millis();
      showingError = true;
      potentiometerToLCD(message);
    }
    if((!showingError || (millis() - lastErrorTime > 1000)) && message.identifier == 0x21){
      showingError = false;
      potentiometerToLCD(message);
    }
  }
  
  // Optional: Check for alerts (errors, etc.)
  uint32_t alerts;
  twai_read_alerts(&alerts, pdMS_TO_TICKS(0));
  if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
    Serial.println("Warning: RX Queue Full!");
  }
}

void printCANMessage(twai_message_t &msg) {
  // Print CAN ID
  Serial.print("ID: 0x");
  if (msg.identifier < 0x100) Serial.print("0");
  if (msg.identifier < 0x10) Serial.print("0");
  Serial.print(msg.identifier, HEX);
  
  // Check if it's extended ID
  if (msg.extd) {
    Serial.print(" (extended)");
  }
  
  // Print data length
  Serial.print("  Len: ");
  Serial.print(msg.data_length_code);
  
  // Print data bytes
  Serial.print("  Data: ");
  for (int i = 0; i < msg.data_length_code; i++) {
    if (msg.data[i] < 0x10) Serial.print("0");
    Serial.print(msg.data[i], HEX);
    Serial.print(" ");
  }
  
  // Print flags if any
  if (msg.rtr) {
    Serial.print(" [RTR]");
  }
  if (msg.ss) {
    Serial.print(" [Single Shot]");
  }
  if (msg.self) {
    Serial.print(" [Self Rx]");
  }
  
  // Print timestamp and count
  Serial.print("  Time: ");
  Serial.print(millis());
  Serial.print(" ms  (#");
  Serial.print(messageCount);
  Serial.println(")");
}

void decodeTWAI(twai_message_t &msg) {
  switch(msg.identifier) {
    case 0x123:
      Serial.print("  → Decoded: Counter = ");
      Serial.println(msg.data[0]);
      
      // If data shows a pattern (like incrementing counter)
      static uint8_t lastValue = 0;
      if (msg.data[0] != lastValue + 1 && lastValue != 0) {
        Serial.print("  ⚠ Warning: Missed message? Expected ");
        Serial.print(lastValue + 1);
        Serial.print(" got ");
        Serial.println(msg.data[0]);
      }
      lastValue = msg.data[0];
      break;
      
    case 0x456:
      // Decode different message format
      if (msg.data_length_code >= 4) {
        uint16_t value1 = (msg.data[1] << 8) | msg.data[0];
        uint16_t value2 = (msg.data[3] << 8) | msg.data[2];
        Serial.print("  → Value1: ");
        Serial.print(value1);
        Serial.print(", Value2: ");
        Serial.println(value2);
      }
      break;
      
    default:
      // Unknown ID
      break;
  }
}

void potentiometerToLCD(twai_message_t message){
  if(message.identifier == 0x020){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERROR");
    lcd.setCursor(0, 1);
    lcd.print("Code: ");
    lcd.print(message.data[0], DEC);
    lcd.print("    ");
  }
  else{
    lcd.setCursor(0, 0);
    lcd.print("Gear: ");
    if (message.data[2] == 0){
      lcd.print("Neutral");
    }
    else if (message.data[2] == 1){
      lcd.print("Drive     ");
    }
    else if(message.data[2] == 2){
      lcd.print("Reverse");
    }
    lcd.setCursor(0, 1);
    lcd.print("Pot: ");
    uint16_t reading = (message.data[0] << 8) | message.data[1];
    if (reading < 1000) {lcd.print("0");}
    if (reading < 100) {lcd.print("0");}
    if (reading < 10) {lcd.print("0");}
    lcd.print(reading);
  }
}