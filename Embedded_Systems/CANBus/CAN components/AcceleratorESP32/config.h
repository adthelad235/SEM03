// ========================================
// Pin definitions
// ========================================
#define CAN_RX_PIN GPIO_NUM_19
#define CAN_TX_PIN GPIO_NUM_21

#define POT1_PIN 34   
#define POT2_PIN 35 

#define GEAR1_PIN 25  
#define GEAR2_PIN 26  

// ========================================
// Period of messages 1/frequency
// ========================================
#define ACCELERATOR_TIME_INTERVAL 20
#define ERROR_TIME_INTERVAL 2000

// ========================================
// Range of accepted potentiometer readings
// ========================================
#define POT_DISCONNECTED_THRESHOLD 4095 // NEEDS CALIBRATED 
#define POT_MIN_VALID 0  // NEEDS CALIBRATED