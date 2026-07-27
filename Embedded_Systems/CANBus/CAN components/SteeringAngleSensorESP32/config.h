#pragma once

// ============================================================
//  PIN CONFIGURATION
// ============================================================

// Mux select pins (digital outputs)
#define MUX_S0_PIN   16
#define MUX_S1_PIN   17
#define MUX_S2_PIN   18
#define MUX_S3_PIN   19

#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

// Mux signal pin
#define MUX_SIG_PIN  34   // GPIO 34 = ADC1 channel 6

// ========================================
// PERIOD OF MESSAGES// ========================================
#define ANGLE_TIME_INTERVAL 10
#define ERROR_TIME_INTERVAL 2000

// ========================================
// STEERING ANGLE
// ========================================
#define CENTRE_POSITION 80.0f // MAY NEED CALIBRATED
#define MAX_STEERING_ANGLE 90.0f // MAY NEED CALIBRATED
#define MAX_DISTANCE_MOVED 80.0f //MAY NEED CALIBRATED

// ============================================================
//  SENSOR ARRAY LAYOUT
// ============================================================
#define N_SENSORS          9
#define SENSOR_SPACING_MM  20.0f   // 2 cm spacing

// Mux channel number for each sensor (0-indexed, left to right)
// Change the order here if your sensors aren't wired sequentially
const uint8_t SENSOR_MUX_CH[N_SENSORS] = {0, 1, 2, 3, 4, 5, 6, 7, 8};

// Physical position of each sensor in mm (from the left end of the array)
const float SENSOR_POS_MM[N_SENSORS] = {
    0.0f, 20.0f, 40.0f, 60.0f, 80.0f, 100.0f, 120.0f, 140.0f, 160.0f
};

// ============================================================
//  TUNING
// ============================================================

// Microseconds to wait after switching mux channel before reading ADC.
// Too low → crosstalk between channels. Too high → wastes timing budget.
#define SETTLE_US          20

// Number of ADC reads to average per sensor per cycle.
#define OVERSAMPLE         4

// Number of reads per sensor during calibrate() call.
#define BASELINE_SAMPLES   64

// ADC value thresholds for fault detection (12-bit ADC = 0 to 4095)
#define RAIL_HIGH_THRESH   4000
#define RAIL_LOW_THRESH    10

// Minimum signal above baseline to be considered a real detection.
#define NOISE_FLOOR        50

// How many consecutive update() calls a sensor must be stuck at a rail
// before it is declared faulty (prevents false faults from noise spikes).
#define STUCK_COUNT        5

// Minimum number of valid sensors required to calculate a position.
// Must be at least 3. More = more robust but less tolerant of failures.
#define MIN_VALID_SENSORS  3

// Minimum peak signal (above baseline) across the whole array to confirm
// a magnet is actually present. Prevents computing a position from noise.
#define MAGNET_DETECT_THRESHOLD  150
