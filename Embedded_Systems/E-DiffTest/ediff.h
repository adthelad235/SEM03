// =============================================================
// ediff.h – E-Differential Algorithm Header (Current Control)
// =============================================================

#ifndef EDIFF_H
#define EDIFF_H

#include "config.h"
#include <cstdint>

// Gear selector states
enum Gear { NEUTRAL = 0, FORWARD = 1, REVERSE = -1 };

// All inbound sensor values
struct SensorData {
    float   steering_deg;       // Degrees. Positive = right, negative = left
    float   throttle_pct;       // 0.0 – 100.0 %
    Gear    gear;               // NEUTRAL / FORWARD / REVERSE
    float   vehicle_speed_ms;   // m/s
    float   yaw_rate_degs;      // deg/s. Positive = turning right
};

// Telemetry from each VESC
struct VescTelemetry {
    int32_t  erpm;              // Electrical RPM (signed)
    float    motor_current_A;   // Motor phase current (amps)
    float    input_current_A;   // Battery current (amps)
    float    duty_cycle;        // Duty cycle 0.0 – 1.0
    float    voltage_V;         // Input voltage
    float    temp_fet_C;        // MOSFET temperature (°C)
    float    temp_motor_C;      // Motor temperature (°C)
    uint8_t  fault_code;        // 0 = no fault
    uint32_t last_update_ms;    // Timestamp
};

// Calculated output commands (current in amps)
struct MotorCommands {
    float left_A;
    float right_A;
};

// Main E-Diff calculation function
MotorCommands calculateDifferential(
    const SensorData& sensors,
    const VescTelemetry& vesc_left,
    const VescTelemetry& vesc_right
);

// Utility: Clamp value to range
float clampValue(float x, float minVal, float maxVal);

#endif // EDIFF_H