//bcole2@sheffield.ac.uk
//clohachala1@sheffield.ac.uk

#include<driver/twai.h>

#define CAN_TX_PIN GPIO_NUM_4 // Transmit pin
#define CAN_RX_PIN GPIO_NUM_5 // Receive pin

// CAN configuration
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_RX_PIN, CAN_TX_PIN, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Initialise the CAN driver
  if(twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK){
    Serial.println("CAN driver installed");
  }
  else{
    Serial.println("CAN driver failed ot install");
    return;
  }

  // Start the CAN driver
  if(twai_start() == ESP_OK){
    Serial.println("CAN driver started up");
  }
  else{
    Serial.println("CAN driver failed to start up");
    return;
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  sendCANMessage();
  delay(1000);
}

bool sendCANMessage(){
  // Configures the message
  twai_message_t message;
  uint32_t myValue = getInput();
  if (myValue == 0){
    return true;
  }
  message.identifier = 0x00000168; // CAN id 
  message.extd = 1; // 0 if 11-bit id, 1 if extended 29-bit id
  message.data_length_code = 4; // 4-bit number of bytes being transmitted in the payload
  message.data[0] = (myValue >> 24) & 0xFF;  // Most significant byte
  message.data[1] = (myValue >> 16) & 0xFF;
  message.data[2] = (myValue >> 8) & 0xFF;
  message.data[3] = myValue & 0xFF;   

  // Transmits message
  return sendMessage(message);
}

uint32_t getInput(){
    if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove whitespace and newline characters
    
    // Check if input is 8 hex digits long
    if (input.length() == 8) {
      // Convert hex string to unsigned 32-bit integer
      uint32_t hexValue = (uint32_t)strtol(input.c_str(), NULL, 16);
      
      // Display results
      Serial.println("\n--- Received ---");
      Serial.print("Hex string: 0x");
      Serial.println(input);
      Serial.print("Decimal value: ");
      Serial.println(hexValue);
      Serial.print("Binary: ");
      Serial.println(hexValue, BIN);
      Serial.println("----------------\n");
      return hexValue;
    } 
    else if (input.length() > 0) {
      Serial.print("Error: '");
      Serial.print(input);
      Serial.println("' is not exactly 8 hex digits. Please try again.");
    }
  }
  return 0;
}

bool sendMessage(twai_message_t& message) {
  unsigned long startTime = millis();
    esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(1000));
    
        if (result == ESP_OK) {
            Serial.print("✅ Message sent - ID: 0x");
            
            // Pad identifier to 3 hex digits
            if (message.identifier < 0x100) Serial.print("0");
            if (message.identifier < 0x10) Serial.print("0");
            Serial.print(message.identifier, HEX);
            
            Serial.print(" Data: ");
            for (int i = 0; i < message.data_length_code; i++) {
                if (message.data[i] < 0x10) Serial.print("0");
                Serial.print(message.data[i], HEX);
                Serial.print(" ");
            }
            
            Serial.print(" (Time: ");
            Serial.print(startTime);
            Serial.println(")");
        } else {
            Serial.print("❌ Failed to send message. Error: 0x");
            Serial.println(result, HEX);
        }
    
    
    return result == ESP_OK;
}


// Key Functions:
// twai_driver_install() - Initialize CAN driver
// twai_start() - Start CAN driver
// twai_transmit() - Send CAN message
// twai_receive() - Receive CAN message
// twai_stop() - Stop CAN driver
// twai_driver_uninstall() - Uninstall CAN driver
