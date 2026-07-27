// Link UART from the hat. RX only (IO38). TX unused (-1).
#define LINK_RX_PIN  38
#define LINK_BAUD    115200

// Display resolution
#define SCREEN_WIDTH  800
#define SCREEN_HEIGHT 480

#define FLASH_SPEED 180

// ---- Warning thresholds (hysteresis: trip at ON, release at CLEAR) ----
#define BATT_LOW_ON        20     // % : warn at/below
#define BATT_LOW_CLEAR     25     // % : clear once above
#define BATT_TEMP_ON       50     // C : warn at/above
#define BATT_TEMP_CLEAR    47     // C : clear below
#define MOTOR_TEMP_ON      80
#define MOTOR_TEMP_CLEAR   75
#define CTRL_TEMP_ON       80
#define CTRL_TEMP_CLEAR    75
#define WARN_TEXT_CYCLE_MS 2000   // how long each msg shows when several active