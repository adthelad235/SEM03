#include "CANHelper.h"

bool CANHelper::setupCAN(gpio_num_t rxPin, gpio_num_t txPin){
    //setting up TWAI

  Serial.println("Setting up esp32 CAN transmitter...");

  //Configure CAN parameters
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)txPin,
    (gpio_num_t)rxPin,
    TWAI_MODE_NORMAL
  );

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  //Install the TWAI driver
  if(twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK){
    Serial.println("TWAI driver installed successfully!");
  }
  else{
    Serial.println("TWAI driver failed to install");
    return false;
  }

  //Start the TWAI driver
  if(twai_start() == ESP_OK){
    Serial.println("TWAI driver started successfully!");
  }
  else{
    Serial.println("TWAI driver failed to start");
    return false;
  }
  return true;
}

bool CANHelper::setupFilterCAN(gpio_num_t rxPin, gpio_num_t txPin) {
    // Initialize configuration structures
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(rxPin, txPin, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        // Start TWAI driver
        if (twai_start() == ESP_OK) {
            return true;
        }
    }
    return false;
}

bool CANHelper::sendMessage(twai_message_t& message, bool debugPrint) {
    unsigned long startTime = millis();
    esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(1000));
    
    if (debugPrint) {
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
    }
    
    return result == ESP_OK;
}

bool CANHelper::sendMessageSilent(twai_message_t& message) {
    return sendMessage(message, false);
}

void CANHelper::printCANMessage(twai_message_t &message, int messageCount) {
  // Print CAN ID
  Serial.print("ID: 0x");
  if (message.identifier < 0x100) Serial.print("0");
  if (message.identifier < 0x10) Serial.print("0");
  Serial.print(message.identifier, HEX);
  
  // Check if it's extended ID
  if (message.extd) {
    Serial.print(" (extended)");
  }
  
  // Print data length
  Serial.print("  Len: ");
  Serial.print(message.data_length_code);
  
  // Print data bytes
  Serial.print("  Data: ");
  for (int i = 0; i < message.data_length_code; i++) {
    if (message.data[i] < 0x10) Serial.print("0");
    Serial.print(message.data[i], HEX);
    Serial.print(" ");
  }
  
  // Print flags if any
  if (message.rtr) {
    Serial.print(" [RTR]");
  }
  if (message.ss) {
    Serial.print(" [Single Shot]");
  }
  if (message.self) {
    Serial.print(" [Self Rx]");
  }
  
  // Print timestamp and count
  Serial.print("  Time: ");
  Serial.print(millis());
  Serial.print(" ms  (#");
  Serial.print(messageCount);
  Serial.println(")");
}

void CANHelper::formatMessage(twai_message_t& message, char* buffer, unsigned long timestamp) {
    char* ptr = buffer;
    
    // Format: "ID:0x123 Data:AA BB CC DD Time:12345"
    ptr += sprintf(ptr, "ID:0x%03X Data:", message.identifier);
    
    for (int i = 0; i < message.data_length_code; i++) {
        ptr += sprintf(ptr, "%02X ", message.data[i]);
    }
    
    if (timestamp > 0) {
        ptr += sprintf(ptr, " Time:%lu", timestamp);
    }
}