#include<driver/twai.h>

#define CAN_TX_PIN GPIO_NUM_21 // Transmit pin
#define CAN_RX_PIN GPIO_NUM_19 // Receive pin

// CAN configuration
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
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
  twai_message_t message;
  esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(1000));

  if (result == ESP_OK) {
    Serial.print(message.extd ? "Ext ID=0x" : "Std ID=0x");
    Serial.print(message.identifier, HEX);

    Serial.print("  DLC=");
    Serial.print(message.data_length_code);

    Serial.print("  Data:");
    if (!message.rtr) {                       // RTR frames carry no data
      for (int i = 0; i < message.data_length_code; i++) {
        Serial.print(" 0x");
        if (message.data[i] < 0x10) Serial.print("0");
        Serial.print(message.data[i], HEX);
      }
    }
    Serial.println();
  }
  // result == ESP_ERR_TIMEOUT just means no frame arrived in 1 s — that's normal
}


