// =============================================================
// main.cpp – Desktop Test Harness for Current Control E-Diff
// =============================================================
// Compile with:
//   g++ -std=c++11 -o ediff_test main.cpp ediff.cpp
// Run with:
//   ./ediff_test
// =============================================================

#include "ediff.h"
#include <cstdio>
#include <cstring>

// Test scenario structure
struct Scenario {
    const char* name;
    float      throttle_pct;
    Gear       gear;
    float      steering_deg;
    float      speed_ms;
    float      yaw_degs;
};

// Predefined test scenarios
const Scenario scenarios[] = {
    { "STOP – neutral, zero throttle", 0.0f, NEUTRAL, 0.0f, 0.0f, 0.0f },
    { "Straight – 25% throttle, forward", 25.0f, FORWARD, 0.0f, 3.0f, 0.0f },
    { "Right turn – 50% throttle, 10° right", 50.0f, FORWARD, 10.0f, 5.0f, 8.0f },
    { "Left turn – 50% throttle, 10° left", 50.0f, FORWARD, -10.0f, 5.0f, -8.0f },
    { "Sharp right – 75% throttle, 25° right", 75.0f, FORWARD, 25.0f, 7.0f, 12.0f },
    { "Sharp left – 75% throttle, 25° left", 75.0f, FORWARD, -25.0f, 7.0f, -12.0f },
    { "Reverse – 25% throttle, straight", 25.0f, REVERSE, 0.0f, 3.0f, 0.0f },
    { "Oversteer – 50% throttle, 10° right, yaw too high", 50.0f, FORWARD, 10.0f, 5.0f, 20.0f },
    { "Understeer – 50% throttle, 10° right, yaw too low", 50.0f, FORWARD, 10.0f, 5.0f, 2.0f },
};

// Simulated VESC telemetry (ideal response, no slip)
VescTelemetry simulateVescTelemetry(float speed_ms, float steering_deg, bool is_left) {
    VescTelemetry telem = {};
    
    // Calculate expected ERPM based on vehicle speed and steering
    float wheel_circ_m = PI * WHEEL_DIAMETER_M;
    float shaft_rpm = (speed_ms / wheel_circ_m) * 60.0f;
    float base_erpm = shaft_rpm * (float)MOTOR_POLE_PAIRS;
    
    // Adjust for steering
    float speed_diff_factor = 0.0f;
    if (fabsf(steering_deg) > 1.0f) {
        float steer_rad = steering_deg * DEG_TO_RAD;
        speed_diff_factor = (WHEEL_TRACK_M * tanf(steer_rad)) / 
                            (2.0f * wheel_circ_m / (2.0f * PI));
    }
    
    if (is_left) {
        telem.erpm = (int32_t)(base_erpm * (1.0f + speed_diff_factor));
    } else {
        telem.erpm = (int32_t)(base_erpm * (1.0f - speed_diff_factor));
    }
    
    telem.motor_current_A = 0.0f;
    telem.voltage_V = 48.0f;
    telem.temp_fet_C = 35.0f;
    telem.temp_motor_C = 35.0f;
    telem.fault_code = 0;
    telem.last_update_ms = 0;
    return telem;
}

void printSeparator() {
    printf("============================================================\n");
}

void runScenario(const Scenario& s) {
    printSeparator();
    printf("SCENARIO: %s\n", s.name);
    printf("------------------------------------------------------------\n");
    printf("INPUTS:\n");
    printf("  Throttle: %.1f%%  |  Gear: %s  |  Steering: %+.1f°\n",
           s.throttle_pct,
           s.gear == FORWARD ? "FWD" : (s.gear == REVERSE ? "REV" : "NEU"),
           s.steering_deg);
    printf("  Speed: %.2f m/s  |  Yaw Rate: %+.2f °/s\n",
           s.speed_ms, s.yaw_degs);

    // Populate sensor struct
    SensorData sensors = {};
    sensors.throttle_pct = s.throttle_pct;
    sensors.gear = s.gear;
    sensors.steering_deg = s.steering_deg;
    sensors.vehicle_speed_ms = s.speed_ms;
    sensors.yaw_rate_degs = s.yaw_degs;

    // Simulate VESC telemetry
    VescTelemetry vesc_left  = simulateVescTelemetry(s.speed_ms, s.steering_deg, true);
    VescTelemetry vesc_right = simulateVescTelemetry(s.speed_ms, s.steering_deg, false);

    // Run the algorithm
    MotorCommands cmd = calculateDifferential(sensors, vesc_left, vesc_right);

    printf("\nOUTPUTS:\n");
    printf("  Left Current:  %+8.2f A\n", cmd.left_A);
    printf("  Right Current: %+8.2f A\n", cmd.right_A);

    // Calculate torque split percentage
    if (cmd.left_A != 0 || cmd.right_A != 0) {
        float avg = (fabsf(cmd.left_A) + fabsf(cmd.right_A)) / 2.0f;
        if (avg > 0.1f) {
            float split_pct = (cmd.left_A - cmd.right_A) / avg * 100.0f;
            printf("  Torque Split: %+.1f%% (positive = left bias)\n", split_pct);
        }
    }
    
    // Show calculated yaw error correction
    float desired_yaw = 0.0f;
    if (fabsf(s.steering_deg) > 1.0f && s.speed_ms > 0.3f) {
        float steer_rad = s.steering_deg * DEG_TO_RAD;
        desired_yaw = (s.speed_ms / WHEEL_TRACK_M) * tanf(steer_rad) * RAD_TO_DEG;
    }
    printf("\nDIAGNOSTICS:\n");
    printf("  Desired Yaw: %.2f °/s\n", desired_yaw);
    printf("  Actual Yaw:  %.2f °/s\n", s.yaw_degs);
    printf("  Yaw Error:   %+.2f °/s\n", desired_yaw - s.yaw_degs);
}

void runSlipTest() {
    printSeparator();
    printf("SLIP TEST – Left Wheel Losing Traction\n");
    printf("------------------------------------------------------------\n");

    SensorData sensors = {};
    sensors.throttle_pct = 50.0f;
    sensors.gear = FORWARD;
    sensors.steering_deg = 0.0f;
    sensors.vehicle_speed_ms = 5.0f;
    sensors.yaw_rate_degs = 0.0f;

    // Left wheel is spinning (slipping on low-grip surface)
    VescTelemetry vesc_left  = simulateVescTelemetry(5.0f, 0.0f, true);
    VescTelemetry vesc_right = simulateVescTelemetry(5.0f, 0.0f, false);
    
    // Artificially increase left wheel ERPM to simulate slip
    vesc_left.erpm = (int32_t)(vesc_left.erpm * 1.5f);  // 50% faster than expected

    MotorCommands cmd = calculateDifferential(sensors, vesc_left, vesc_right);

    printf("Left wheel ERPM: %ld (SPINNING - should trigger TC)\n", (long)vesc_left.erpm);
    printf("Right wheel ERPM: %ld (normal)\n", (long)vesc_right.erpm);
    printf("\nOUTPUTS (Traction Control should reduce left current):\n");
    printf("  Left Current:  %+8.2f A\n", cmd.left_A);
    printf("  Right Current: %+8.2f A\n", cmd.right_A);
}

void runHighSpeedTest() {
    printSeparator();
    printf("HIGH SPEED TEST – Current Limiting at Top Speed\n");
    printf("------------------------------------------------------------\n");

    SensorData sensors = {};
    sensors.throttle_pct = 100.0f;
    sensors.gear = FORWARD;
    sensors.steering_deg = 0.0f;
    sensors.vehicle_speed_ms = 20.0f;  // 72 km/h
    sensors.yaw_rate_degs = 0.0f;

    VescTelemetry vesc_left  = simulateVescTelemetry(20.0f, 0.0f, true);
    VescTelemetry vesc_right = simulateVescTelemetry(20.0f, 0.0f, false);

    MotorCommands cmd = calculateDifferential(sensors, vesc_left, vesc_right);

    printf("Vehicle Speed: %.1f m/s (%.1f km/h)\n", 20.0f, 20.0f * 3.6f);
    printf("Average ERPM: %.0f\n", (vesc_left.erpm + vesc_right.erpm) / 2.0f);
    printf("\nOUTPUTS (Current should be reduced due to high speed):\n");
    printf("  Left Current:  %+8.2f A\n", cmd.left_A);
    printf("  Right Current: %+8.2f A\n", cmd.right_A);
}

int main() {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║   E-DIFFERENTIAL ALGORITHM – CURRENT CONTROL TEST HARNESS  ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n");
    printf("\nConfiguration:\n");
    printf("  MAX_MOTOR_CURRENT_A: %.0f A\n", MAX_MOTOR_CURRENT_A);
    printf("  STEERING_MAX_DEG: %.1f°\n", STEERING_MAX_DEG);
    printf("  ACKERMANN_GAIN: %.2f\n", ACKERMANN_GAIN);
    printf("  YAW_CORRECTION_GAIN: %.2f A/°\n", YAW_CORRECTION_GAIN);
    printf("  WHEEL_TRACK_M: %.2f m\n", WHEEL_TRACK_M);
    printf("  WHEEL_DIAMETER_M: %.2f m\n", WHEEL_DIAMETER_M);
    printf("  MOTOR_POLE_PAIRS: %d\n", MOTOR_POLE_PAIRS);

    // Run all predefined scenarios
    int num_scenarios = sizeof(scenarios) / sizeof(scenarios[0]);
    for (int i = 0; i < num_scenarios; i++) {
        runScenario(scenarios[i]);
    }

    // Run special tests
    runSlipTest();
    runHighSpeedTest();

    printSeparator();
    printf("All tests completed.\n");
    printSeparator();

    return 0;
}