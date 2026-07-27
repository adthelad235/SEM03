#include <driver/twai.h>

// Pin definitions
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("ESP32 CAN Transmitter (TWAI) Starting...");
  
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
}

void loop() {
  // Prepare CAN message
  twai_message_t message;
  message.identifier = 0x123;    // CAN ID
  message.extd = 0;               // Standard 11-bit ID
  message.data_length_code = 8;   // Data length
  
  // Fill with test data
  static uint8_t counter = 0;
  message.data[0] = 0x01;
  message.data[1] = 0x02;
  message.data[2] = 0x03;
  message.data[3] = 0x04;
  message.data[4] = 0x05;
  message.data[5] = 0x06;
  message.data[6] = 0x07;
  message.data[7] = 0x08;
  
  // Transmit message
  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.print("Message sent! ID: 0x");
    Serial.print(message.identifier, HEX);
    Serial.print(" Data: ");
    for (int i = 0; i < message.data_length_code; i++) {
      Serial.print("0x");
      Serial.print(message.data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.println("Failed to send message");
  }
  
  delay(1000); // Send every second
}