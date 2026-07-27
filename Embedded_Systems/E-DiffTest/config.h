// =============================================================
// config.h – Desktop Test Configuration (Current Control Mode)
// =============================================================

#ifndef CONFIG_H
#define CONFIG_H

#include <cmath>
#include <cstdint>

// ── Math constants (for desktop) ─────────────────────────────
#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)

// ── Motor geometry ───────────────────────────────────────────
const int   MOTOR_POLE_PAIRS    = 7;
const float WHEEL_DIAMETER_M    = 0.60f;
const float WHEEL_TRACK_M       = 1.40f;

// ── Throttle ADC calibration ─────────────────────────────────
const int   THROTTLE_ADC_MIN    = 0;
const int   THROTTLE_ADC_MAX    = 4095;

// ── Steering calibration ─────────────────────────────────────
const float STEERING_MAX_DEG    = 30.0f;

// ── Motor current limits ─────────────────────────────────────
const float MAX_MOTOR_CURRENT_A = 200.0f;   // Max amps per motor

// ── E-differential tuning ────────────────────────────────────
const float ACKERMANN_GAIN      = 0.25f;    // Slightly higher for current control

// ── Torque-vectoring (yaw correction) ────────────────────────
const float YAW_CORRECTION_GAIN = 2.0f;     // Amps per degree of yaw error

// ── Traction control ─────────────────────────────────────────
const float SLIP_THRESHOLD_ERPM = 350.0f;   // ERPM slip threshold
const float TRACTION_CTRL_FACTOR = 0.70f;   // Current reduction factor

// ── Safety limits ────────────────────────────────────────────
const float MOTOR_TEMP_LIMIT_C   = 80.0f;
const float FET_TEMP_LIMIT_C     = 80.0f;

// ── Control loop rate ────────────────────────────────────────
const uint32_t LOOP_PERIOD_MS    = 10;

#endif // CONFIG_H