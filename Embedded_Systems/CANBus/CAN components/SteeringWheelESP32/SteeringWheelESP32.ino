#include "config.h"
#include <driver/twai.h>
#include <CANHelper.h>

// Task handle
TaskHandle_t mainTaskHandle = NULL;

// Move global variables inside task or make them static
void mainTask(void* parameter) {
    static unsigned long previousLightsTime = 0;
    static unsigned long previousErrorTime = 0;
    static unsigned long lastStatus = 0;
    
    Serial.println("Main task started with 8KB stack");
    
    while (true) {
        uint8_t data = getButtons();
        unsigned long currentTime = millis();
        
        if((data & 0x07) && (currentTime - previousErrorTime) >= ERROR_MESSAGE_INTERVAL){
            previousErrorTime = currentTime;
            sendErrorMessage(data);
        }
        else if((currentTime - previousLightsTime) >= LIGHTS_MESSAGE_INTERVAL){
            previousLightsTime = currentTime;
            sendLightsMessage(data);
        }
        
        if (millis() - lastStatus >= 1000) {   // once per second
        lastStatus = millis();
        printCanStatus();
        }
        
        // Small delay to yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("Starting Steering Wheel Controller...");
    
    //setting up pins
    pinMode(HAZARDS_PIN, INPUT_PULLDOWN);
    pinMode(RIGHT_IND_PIN, INPUT_PULLDOWN);
    pinMode(LEFT_IND_PIN, INPUT_PULLDOWN);
    pinMode(FULL_BEAM_PIN, INPUT_PULLDOWN);
    pinMode(HORN_PIN, INPUT_PULLDOWN);

    // Sets up TWAI driver
    if (!CANHelper::setupCAN(CAN_RX_PIN, CAN_TX_PIN)) {
        Serial.println("CAN initialization failed!");
    }
    
    // Create main task with 8KB stack (double the default 4KB)
    xTaskCreate(
        mainTask,           // Task function
        "MainTask",         // Name (for debugging)
        8192,               // Stack size in bytes (8KB)
        NULL,               // Parameters
        1,                  // Priority (1 = normal)
        &mainTaskHandle     // Task handle
    );
    
    // Check if task creation succeeded
    if (mainTaskHandle == NULL) {
        Serial.println("ERROR: Failed to create main task!");
    } else {
        Serial.println("Main task created successfully");
        Serial.printf("Task stack size: %d bytes\n", 8192);
    }
}

void loop() {
    // Empty - mainTask handles everything
    // Just delay forever
    vTaskDelay(portMAX_DELAY);
}

uint8_t getButtons(){
    uint8_t data = 0;
    if(digitalRead(HAZARDS_PIN)){
        data += 128;
    }
    if(digitalRead(RIGHT_IND_PIN)){
        data += 64;
    }
    if(digitalRead(LEFT_IND_PIN)){
        data += 32;
    }
    if(digitalRead(FULL_BEAM_PIN)){
        data += 16;
    }
    if(digitalRead(HORN_PIN)){
        data += 8;
    }
    return data;
}

bool sendLightsMessage(uint8_t data){
    twai_message_t message = {0};
    message.identifier = 0x075;
    message.extd = 0;
    message.data_length_code = 1;
    message.data[0] = data;
    return CANHelper::sendMessage(message);
}

bool sendErrorMessage(uint8_t data){
    twai_message_t message = {0};
    message.identifier = 0x074;
    message.extd = 0;
    message.data_length_code = 1;
    message.data[0] = data;
    return CANHelper::sendMessage(message);
}

void printCanStatus() {
  twai_status_info_t s;
  if (twai_get_status_info(&s) != ESP_OK) {
    Serial.println("twai_get_status_info() failed");
    return;
  }

  const char* stateStr;
  switch (s.state) {
    case TWAI_STATE_STOPPED:    stateStr = "STOPPED";    break;
    case TWAI_STATE_RUNNING:    stateStr = "RUNNING";    break;
    case TWAI_STATE_BUS_OFF:    stateStr = "BUS_OFF";    break;
    case TWAI_STATE_RECOVERING: stateStr = "RECOVERING"; break;
    default:                    stateStr = "UNKNOWN";    break;
  }

  Serial.println("---- TWAI status ----");
  Serial.printf("State            : %s\n",  stateStr);
  Serial.printf("TX error counter : %u\n",  s.tx_error_counter);
  Serial.printf("RX error counter : %u\n",  s.rx_error_counter);
  Serial.printf("Msgs queued TX   : %u\n",  s.msgs_to_tx);
  Serial.printf("Msgs queued RX   : %u\n",  s.msgs_to_rx);
  Serial.printf("TX failed count  : %u\n",  s.tx_failed_count);
  Serial.printf("RX missed count  : %u\n",  s.rx_missed_count);
  Serial.printf("RX overrun count : %u\n",  s.rx_overrun_count);
  Serial.printf("Arb lost count   : %u\n",  s.arb_lost_count);
  Serial.printf("Bus error count  : %u\n",  s.bus_error_count);
  Serial.println("---------------------");
}
