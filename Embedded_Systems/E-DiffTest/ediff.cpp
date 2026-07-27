// =============================================================
// ediff.cpp – E-Differential Algorithm (Current Control Mode)
// =============================================================
// This version commands MOTOR CURRENT (torque) instead of RPM.
// Current is proportional to acceleration force, making the
// vehicle feel natural like a conventional accelerator pedal.
// =============================================================

#include "ediff.h"
#include <cstdio>

float clampValue(float x, float minVal, float maxVal) {
    if (x < minVal) return minVal;
    if (x > maxVal) return maxVal;
    return x;
}

MotorCommands calculateDifferential(
    const SensorData& sensors,
    const VescTelemetry& vesc_left,
    const VescTelemetry& vesc_right)
{
    MotorCommands cmd = { 0.0f, 0.0f };

    // Nothing to do while stopped or in neutral
    if (sensors.gear == NEUTRAL || sensors.throttle_pct < 0.5f) {
        return cmd;
    }

    // ── 1. Base current from throttle ────────────────────────
    // Throttle pedal maps directly to torque demand
    float base_current = (sensors.throttle_pct / 100.0f) * MAX_MOTOR_CURRENT_A;

    // ── 2. Ackermann differential split ──────────────────────
    // During a turn, the outside wheel needs more torque to push
    // the vehicle through the corner (it travels a longer arc).
    float turn_factor = (sensors.steering_deg / STEERING_MAX_DEG) * ACKERMANN_GAIN;

    // Positive steering_deg = turning right:
    //   right wheel = inside (needs less torque), left wheel = outside (needs more)
    float left_current  = base_current * (1.0f + turn_factor);
    float right_current = base_current * (1.0f - turn_factor);

    // ── 3. Yaw-rate torque vectoring ─────────────────────────
    // Calculate the yaw rate we'd expect from steering geometry alone.
    float desired_yaw = 0.0f;
    if (fabsf(sensors.steering_deg) > 1.0f && sensors.vehicle_speed_ms > 0.3f) {
        float steer_rad = sensors.steering_deg * DEG_TO_RAD;
        desired_yaw = (sensors.vehicle_speed_ms / WHEEL_TRACK_M) *
                       tanf(steer_rad) * RAD_TO_DEG;
    }

    float yaw_error      = desired_yaw - sensors.yaw_rate_degs;
    float yaw_correction = yaw_error * YAW_CORRECTION_GAIN;

    // Positive yaw_error = understeering (car not turning enough)
    // Add torque to outer (left) wheel to help rotate the car
    left_current  += yaw_correction;
    right_current -= yaw_correction;

    // ── 4. Traction control (slip detection) ─────────────────
    // If a wheel is spinning faster than vehicle speed suggests,
    // it has lost traction. Reduce current to that wheel.
    float wheel_circ_m = PI * WHEEL_DIAMETER_M;
    float expected_shaft_rpm = (sensors.vehicle_speed_ms / wheel_circ_m) * 60.0f;
    float expected_erpm      = expected_shaft_rpm * (float)MOTOR_POLE_PAIRS;

    // Adjust expected ERPM for each wheel based on steering geometry
    float speed_diff_factor = 0.0f;
    if (fabsf(sensors.steering_deg) > 1.0f) {
        float steer_rad = sensors.steering_deg * DEG_TO_RAD;
        speed_diff_factor = (WHEEL_TRACK_M * tanf(steer_rad)) / 
                            (2.0f * wheel_circ_m / (2.0f * PI));
    }

    float expected_left_erpm  = expected_erpm * (1.0f + speed_diff_factor);
    float expected_right_erpm = expected_erpm * (1.0f - speed_diff_factor);

    // Use absolute ERPM for comparison (handles both forward and reverse)
    bool left_slip  = (fabsf((float)vesc_left.erpm)  > expected_left_erpm  + SLIP_THRESHOLD_ERPM);
    bool right_slip = (fabsf((float)vesc_right.erpm) > expected_right_erpm + SLIP_THRESHOLD_ERPM);

    if (left_slip) {
        left_current *= TRACTION_CTRL_FACTOR;
        printf("TC: Left wheel slip detected (ERPM: %ld, Expected: %.0f)\n", 
               (long)vesc_left.erpm, expected_left_erpm);
    }
    if (right_slip) {
        right_current *= TRACTION_CTRL_FACTOR;
        printf("TC: Right wheel slip detected (ERPM: %ld, Expected: %.0f)\n", 
               (long)vesc_right.erpm, expected_right_erpm);
    }

    // ── 5. Speed-based current limiting ──────────────────────
    // At high speeds, reduce maximum allowed current to prevent
    // over-speeding the motors (field weakening protection)
    float speed_factor = 1.0f;
    float avg_erpm = (fabsf((float)vesc_left.erpm) + fabsf((float)vesc_right.erpm)) / 2.0f;
    if (avg_erpm > 15000.0f) {
        // Linearly reduce current above 15000 ERPM
        speed_factor = 1.0f - clampValue((avg_erpm - 15000.0f) / 5000.0f, 0.0f, 0.5f);
        left_current  *= speed_factor;
        right_current *= speed_factor;
    }

    // ── 6. Direction and safety limits ───────────────────────
    float direction = (sensors.gear == REVERSE) ? -1.0f : 1.0f;
    cmd.left_A  = clampValue(left_current  * direction, -MAX_MOTOR_CURRENT_A, MAX_MOTOR_CURRENT_A);
    cmd.right_A = clampValue(right_current * direction, -MAX_MOTOR_CURRENT_A, MAX_MOTOR_CURRENT_A);

    return cmd;
}