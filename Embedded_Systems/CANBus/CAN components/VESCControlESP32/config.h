// ==============================
// MAIN CONTROL PARAMETERS
// ==============================
const float MAX_OUTPUT = 20000.0f;   // Maximum magnitude of wheel command NEEDS CALIBRATED
const float STEER_GAIN = 0.5f;    // Steering gain controls ediff split strength NEEDS CALIBRATED

// ==============================
// ADC SETTINGS
// ==============================
const int ADC_MAX = 4095;         // NEEDS CALIBRATED
const int ADC_MIN = 0;            // NEEDS CALIBRATED

// =======================================
// STEERING SENSOR CALIBRATION
// =======================================
const float MAX_STEER_ANGLE_DEG = 30.0f; // NEEDS CALIBRATED

// =============================
// PIN DEFINITIONS
// =============================
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4
#define VESC1_TX_PIN GPIO_NUM_17
#define VESC1_RX_PIN GPIO_NUM_16
#define VESC2_TX_PIN GPIO_NUM_26
#define VESC2_RX_PIN GPIO_NUM_25

// ===============================
// DIRECTION STATES
// ===============================
enum DirectionState {
  NEUTRAL = 0,
  FORWARD = 1,
  REVERSE = -1
};