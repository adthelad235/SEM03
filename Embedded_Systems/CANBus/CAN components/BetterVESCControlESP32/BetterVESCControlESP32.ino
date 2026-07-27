// =============================================================
// EDiff_VESC_ESP32.ino
// E-Differential controller for two Trampa VESC 100/250s
// Platform: ESP32
// Inputs:   CAN bus  – throttle/gear, steering, speed, IMU yaw rate
// Outputs:  UART     – current commands to left and right VESC
// Feedback: UART     – ERPM, temperatures, voltage, fault codes
// =============================================================

#include "config.h"
#include <driver/twai.h>   // ESP32 built-in TWAI (CAN) driver
#include <CANHelper.h>

// =============================================================
// SECTION 1 – DATA TYPES
// =============================================================

// Gear selector states
enum Gear { NEUTRAL = 0, FORWARD = 1, REVERSE = -1 };

// All inbound sensor values live here.
// Populated every loop from CAN messages and VESC replies.
struct SensorData {
    float   steering_deg;       // Degrees. Positive = right, negative = left
    float   throttle_pct;       // 0.0 – 100.0 %
    Gear    gear;               // NEUTRAL / FORWARD / REVERSE
    float   vehicle_speed_ms;   // m/s. Derived from CAN INT16 cm/s message
    float   yaw_rate_degs;      // deg/s. Positive = turning right (CAN INT16 * 0.01)
};

// Telemetry read back from each VESC over UART.
struct VescTelemetry {
    int32_t  erpm;              // Electrical RPM (signed). Divide by MOTOR_POLE_PAIRS for shaft RPM
    float    motor_current_A;   // Actual motor phase current (amps)
    float    input_current_A;   // Battery/input current draw (amps)
    float    duty_cycle;        // Duty cycle 0.0 – 1.0
    float    voltage_V;         // Input voltage (volts)
    float    temp_fet_C;        // MOSFET temperature (°C)
    float    temp_motor_C;      // Motor temperature (°C)
    uint8_t  fault_code;        // 0 = no fault
    uint32_t last_update_ms;    // millis() timestamp of last successful parse
};

// Calculated output currents for the two motors
struct MotorCommands {
    float left_A;
    float right_A;
};

// =============================================================
// SECTION 2 – GLOBAL STATE
// =============================================================

SensorData   sensors;
VescTelemetry vesc_left;
VescTelemetry vesc_right;

bool     emergency_stop    = false;
int error_last_sent[66] = {0,0,0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,0,0,0,
                         0,0,0,0,0,0};

uint32_t last_can_rx_ms    = 0;   // Time of most recent CAN message received

// UART hardware serials – ESP32 has three hardware UARTs (0,1,2)
// Serial  = UART0 (USB debug)
// Serial1 = UART1 → Left VESC
// Serial2 = UART2 → Right VESC
HardwareSerial VescLeft(1);
HardwareSerial VescRight(2);

// =============================================================
// SECTION 3 – CRC-16 (VESC standard)
// =============================================================
// The VESC uses a specific CRC-16 polynomial (0xA001 / IBM variant).
// Every outgoing packet payload is checksummed with this before sending.

static uint16_t crc16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// =============================================================
// SECTION 4 – VESC UART PACKET HELPERS
// =============================================================
// The VESC packet frame is:
//  [0x02] [length 1B] [payload N bytes] [CRC high] [CRC low] [0x03]
// For payloads > 255 bytes the start byte is 0x03 and length is 2 bytes,
// but we never send packets that large so 0x02 is always used here.

// Send a raw payload over the given serial port.
static void vescSendPacket(HardwareSerial& port, const uint8_t* payload, uint16_t len) {
    uint16_t crc = crc16(payload, len);

    port.write(0x02);                      // Short-packet start byte
    port.write((uint8_t)len);              // Payload length
    port.write(payload, len);             // Payload
    port.write((uint8_t)(crc >> 8));      // CRC high byte
    port.write((uint8_t)(crc & 0xFF));    // CRC low byte
    port.write(0x03);                      // Stop byte
}

// COMM_SET_CURRENT (command 6)
// Sends a signed current command in amps to the specified VESC.
// Positive = motoring in the configured direction, negative = braking/regen.
// In REVERSE gear the calling code negates the value before passing it here.
static void vescSetCurrent(HardwareSerial& port, float amps) {
    int32_t milliamps = (int32_t)(amps * 1000.0f);
    uint8_t payload[5];
    payload[0] = 6;                                 // COMM_SET_CURRENT
    payload[1] = (milliamps >> 24) & 0xFF;
    payload[2] = (milliamps >> 16) & 0xFF;
    payload[3] = (milliamps >> 8)  & 0xFF;
    payload[4] =  milliamps        & 0xFF;
    vescSendPacket(port, payload, 5);
}

// COMM_SET_CURRENT_BRAKE (command 7)
// Applies regenerative braking at the specified current.
static void vescSetBrakeCurrent(HardwareSerial& port, float amps) {
    int32_t milliamps = (int32_t)(amps * 1000.0f);
    uint8_t payload[5];
    payload[0] = 7;                                 // COMM_SET_CURRENT_BRAKE
    payload[1] = (milliamps >> 24) & 0xFF;
    payload[2] = (milliamps >> 16) & 0xFF;
    payload[3] = (milliamps >> 8)  & 0xFF;
    payload[4] =  milliamps        & 0xFF;
    vescSendPacket(port, payload, 5);
}

// COMM_GET_VALUES (command 4)
// Requests the full telemetry response (~63 bytes) from the VESC.
// The reply is read back asynchronously in parseVescResponse().
static void vescRequestTelemetry(HardwareSerial& port) {
    uint8_t payload[1] = { 4 };   // COMM_GET_VALUES
    vescSendPacket(port, payload, 1);
}

// =============================================================
// SECTION 5 – VESC UART RESPONSE PARSER
// =============================================================
// COMM_GET_VALUES response payload layout (after the 0x04 command byte):
// Offset  Size  Scale   Field
//  0       2     /10    temp_fet         (°C)
//  2       2     /10    temp_motor       (°C)
//  4       4     /100   avg_motor_current (A)
//  8       4     /100   avg_input_current (A)
// 12       4     /100   avg_id  (FOC d-axis)
// 16       4     /100   avg_iq  (FOC q-axis)
// 20       2     /1000  duty_now  (0–1)
// 22       4     /1     erpm  (signed INT32)
// 26       2     /10    v_in  (V)
// 28       4     /10000 amp_hours
// 32       4     /10000 amp_hours_charged
// 36       4     /10000 watt_hours
// 40       4     /10000 watt_hours_charged
// 44       4     /1     tachometer
// 48       4     /1     tachometer_abs
// 52       1            fault_code
// Total payload = 53 bytes → full packet = 58 bytes
//
// Full packet on the wire: 02 3A 04 [53 bytes data] [CRC hi] [CRC lo] 03
//  = 1+1+1+53+2+1 = 59 bytes  (0x3A = 58, but that is payload+cmd = 54? see note)
// NOTE: The length byte counts the payload INCLUDING the command byte.
// payload = cmd(1) + data(52) = 53 bytes → length byte = 0x35 = 53.
// Total wire bytes = 1+1+53+2+1 = 58.

static bool parseVescResponse(HardwareSerial& port, VescTelemetry& telem) {
    // Need at least 58 bytes for a complete COMM_GET_VALUES reply
    if (port.available() < 58) return false;

    // Peek at start byte – if it's not 0x02 drain until we find one or give up
    while (port.available() > 0 && port.peek() != 0x02) {
        port.read();
    }
    if (port.available() < 58) return false;

    uint8_t buf[64];
    int n = port.readBytes(buf, 58);
    if (n < 58) return false;

    // Validate frame
    if (buf[0] != 0x02) return false;               // Start byte
    if (buf[57] != 0x03) return false;              // Stop byte
    uint8_t pay_len = buf[1];                        // Should be 53 (0x35)
    if (pay_len > 54) return false;                  // Sanity check

    // CRC check over the payload bytes (buf[2] … buf[2+pay_len-1])
    uint16_t rx_crc = ((uint16_t)buf[2 + pay_len] << 8) | buf[2 + pay_len + 1];
    uint16_t calc_crc = crc16(&buf[2], pay_len);
    if (rx_crc != calc_crc) return false;

    // Command byte should be 0x04 (COMM_GET_VALUES)
    if (buf[2] != 0x04) return false;

    // Parse fields (buf[3] is first data byte after command byte)
    const uint8_t* d = &buf[3];

    telem.temp_fet_C     = (float)((int16_t)((d[0] << 8) | d[1])) / 10.0f;
    telem.temp_motor_C   = (float)((int16_t)((d[2] << 8) | d[3])) / 10.0f;

    int32_t avg_motor_i  = (int32_t)((d[4]<<24)|(d[5]<<16)|(d[6]<<8)|d[7]);
    telem.motor_current_A = (float)avg_motor_i / 100.0f;

    int32_t avg_input_i  = (int32_t)((d[8]<<24)|(d[9]<<16)|(d[10]<<8)|d[11]);
    telem.input_current_A = (float)avg_input_i / 100.0f;

    // Skip avg_id (4B) and avg_iq (4B) – offsets 12-19
    int16_t duty_raw     = (int16_t)((d[20] << 8) | d[21]);
    telem.duty_cycle     = (float)duty_raw / 1000.0f;

    int32_t erpm_raw     = (int32_t)((d[22]<<24)|(d[23]<<16)|(d[24]<<8)|d[25]);
    telem.erpm           = erpm_raw;

    int16_t vin_raw      = (int16_t)((d[26] << 8) | d[27]);
    telem.voltage_V      = (float)vin_raw / 10.0f;

    // Skip amp_hours (4B), amp_hours_charged (4B), watt_hours (4B),
    //       watt_hours_charged (4B), tachometer (4B), tachometer_abs (4B)
    // Those land at data offsets 28-51

    telem.fault_code     = d[52];
    telem.last_update_ms = millis();

    return true;
}

// Decode a throttle + gear CAN message into the global sensors struct.
// byte 0-1: raw ADC INT16 (0 – ADC_MAX)
// byte 2:   gear (0=Neutral, 1=Forward, 2=Reverse)
static void decodeThrottle(const twai_message_t& msg) {
    int16_t raw = (int16_t)((msg.data[0] << 8) | msg.data[1]);
    sensors.throttle_pct = ((float)(raw - THROTTLE_ADC_MIN) /
                            (float)(THROTTLE_ADC_MAX - THROTTLE_ADC_MIN)) * 100.0f;
    sensors.throttle_pct = constrain(sensors.throttle_pct, 0.0f, 100.0f);

    uint8_t g = msg.data[2];
    if      (g == 1) sensors.gear = FORWARD;
    else if (g == 2) sensors.gear = REVERSE;
    else             sensors.gear = NEUTRAL;
}

// Decode a steering angle CAN message.
// byte 0-1: INT16 = angle * 100 (e.g. 1500 → 15.00°)
// Positive = right, negative = left.
static void decodeSteering(const twai_message_t& msg) {
    int16_t raw = (int16_t)((msg.data[0] << 8) | msg.data[1]);
    sensors.steering_deg = (float)raw / 100.0f;
}

// Decode IMU yaw rate CAN message.
// byte 0-1: INT16 = yaw_rate * 100 in deg/s
// Positive = turning right (clockwise from above), negative = left.
static void decodeIMU(const twai_message_t& msg) {
    int16_t raw = (int16_t)((msg.data[0] << 8) | msg.data[1]);
    sensors.vehicle_speed_ms = fabsf((float)raw / 100.0f)
    raw = (int16_t)((msg.data[2] << 8) | msg.data[3]);
    sensors.yaw_rate_degs = (float)raw / 100.0f;
}

// Non-blocking CAN receive pass.
// Call every loop iteration; processes all messages currently in the queue.
static void processCAN() {
    twai_message_t msg;
    while (twai_receive(&msg, pdMS_TO_TICKS(0)) == ESP_OK) {
        last_can_rx_ms = millis();

        switch (msg.identifier) {
            case CAN_ID_THROTTLE:  decodeThrottle(msg);  break;
            case CAN_ID_STEERING:  decodeSteering(msg);  break;
            case CAN_ID_SPEED:     decodeSpeed(msg);     break;
            case CAN_ID_IMU:       decodeIMU(msg);       break;
            default: break;   // Ignore unknown IDs
        }
    }
}

// =============================================================
// SECTION 7 – VESC TELEMETRY POLLING
// =============================================================
// Requests and parses telemetry from both VESCs.
// Requests are staggered by half the rate period so both VESCs
// aren't asked simultaneously (avoids brief UART contention).

static void updateVescTelemetry() {
    static uint32_t last_req_ms = 0;
    static bool     req_phase   = false;  // false = ask left, true = ask right

    uint32_t now = millis();
    if (now - last_req_ms >= VESC_TELEMETRY_RATE_MS / 2) {
        last_req_ms = now;
        if (!req_phase) {
            vescRequestTelemetry(VescLeft);
        } else {
            vescRequestTelemetry(VescRight);
        }
        req_phase = !req_phase;
    }

    // Try to parse any waiting replies (non-blocking)
    parseVescResponse(VescLeft,  vesc_left);
    parseVescResponse(VescRight, vesc_right);
}

// =============================================================
// SECTION 8 – E-DIFFERENTIAL CALCULATION
// =============================================================
// Inputs:  sensors (steering, throttle, gear, speed, yaw_rate)
//          vesc_left.erpm, vesc_right.erpm (for slip detection)
// Outputs: MotorCommands with left_A and right_A
//
// Stages:
//  1. Base current from throttle
//  2. Ackermann differential split (geometry-based)
//  3. Yaw-rate torque vectoring correction
//  4. Traction control (ERPM-based slip detection)
//  5. Safety limits and gear direction

static MotorCommands calculateDifferential() {
    MotorCommands cmd = { 0.0f, 0.0f };

    // Nothing to do while stopped or in neutral
    if (sensors.gear == NEUTRAL || sensors.throttle_pct < 0.5f) {
        return cmd;
    }

    // ── 1. Base current ───────────────────────────────────────
    float base_A = (sensors.throttle_pct / 100.0f) * MAX_MOTOR_CURRENT_A;

    // ── 2. Ackermann differential split ──────────────────────
    // Outside wheel travels a longer arc and needs more torque/speed.
    // turn_factor is proportional to steering angle, normalised to max angle.
    float turn_factor = (sensors.steering_deg / STEERING_MAX_DEG) * ACKERMANN_GAIN;

    // Positive steering_deg = turning right:
    //   right wheel = inside (slower), left wheel = outside (faster)
    float left_A  = base_A * (1.0f + turn_factor);
    float right_A = base_A * (1.0f - turn_factor);

    // ── 3. Yaw-rate torque vectoring ─────────────────────────
    // Calculate the yaw rate we'd expect from steering geometry alone.
    // yaw ≈ v * tan(δ) / L   where L ≈ track width (simplified for skid-steer)
    float desired_yaw = 0.0f;
    if (fabsf(sensors.steering_deg) > 1.0f && sensors.vehicle_speed_ms > 0.3f) {
        float steer_rad = sensors.steering_deg * DEG_TO_RAD;
        desired_yaw = (sensors.vehicle_speed_ms / WHEEL_TRACK_M) *
                       tanf(steer_rad) * RAD_TO_DEG;
    }

    float yaw_error      = desired_yaw - sensors.yaw_rate_degs;
    float yaw_correction = yaw_error * YAW_CORRECTION_GAIN;

    // Positive yaw_error = understeering; add torque to outer (left) wheel
    left_A  += yaw_correction;
    right_A -= yaw_correction;

    // ── 4. Traction control (ERPM-based) ─────────────────────
    // Convert vehicle speed to expected shaft RPM, then to expected ERPM.
    // expected_shaft_rpm = (speed_ms / circumference) * 60
    // expected_erpm      = shaft_rpm * MOTOR_POLE_PAIRS
    float wheel_circ_m = PI * WHEEL_DIAMETER_M;
    float expected_shaft_rpm = (sensors.vehicle_speed_ms / wheel_circ_m) * 60.0f;
    float expected_erpm      = expected_shaft_rpm * (float)MOTOR_POLE_PAIRS;

    // Adjust expected ERPM for each wheel based on steering geometry
    float speed_diff_factor = 0.0f;
    if (fabsf(sensors.steering_deg) > 1.0f) {
        float steer_rad = sensors.steering_deg * DEG_TO_RAD;
        speed_diff_factor = (WHEEL_TRACK_M * tanf(steer_rad)) / (2.0f * wheel_circ_m / (2.0f * PI));
    }

    float expected_left_erpm  = expected_erpm * (1.0f + speed_diff_factor);
    float expected_right_erpm = expected_erpm * (1.0f - speed_diff_factor);

    // Use absolute ERPM for comparison (handles both forward and reverse)
    bool left_slip  = (fabsf((float)vesc_left.erpm)  > expected_left_erpm  + SLIP_THRESHOLD_ERPM);
    bool right_slip = (fabsf((float)vesc_right.erpm) > expected_right_erpm + SLIP_THRESHOLD_ERPM);

    if (left_slip) {
        left_A *= TRACTION_CTRL_FACTOR;
        Serial.println("TC: Left wheel slip");
    }
    if (right_slip) {
        right_A *= TRACTION_CTRL_FACTOR;
        Serial.println("TC: Right wheel slip");
    }

    // ── 5. Direction and safety limits ───────────────────────
    float direction = (sensors.gear == REVERSE) ? -1.0f : 1.0f;
    if (sensors.gear == FORWARD){
        cmd.left_A  = constrain(left_A, 0.0f, MAX_MOTOR_CURRENT_A);
        cmd.right_A = constrain(right_A, 0.0f, MAX_MOTOR_CURRENT_A);
    }
    if (sensors.gear == REVERSE){
        cmd.left_A  = constrain(-left_A, -MAX_MOTOR_CURRENT_A, 0.0f);
        cmd.right_A = constrain(-right_A, -MAX_MOTOR_CURRENT_A, 0.0f);
    }
    return cmd;
}

// =============================================================
// SECTION 9 – SAFETY WATCHDOG
// =============================================================
// Checks CAN timeouts, VESC reply timeouts, temperatures, and fault codes.
// Sets emergency_stop = true on any violation.

static void checkSafety() {
    uint32_t now = millis();

    // CAN bus timeout – no messages received recently
    if (now - last_can_rx_ms > CAN_TIMEOUT_MS) {
        Serial.println("SAFETY: CAN timeout – no messages received");
        emergency_stop = true;
        if (now - error_last_sent[59] > CAN_ERROR_MESSAGE_MS){
            error_last_sent[59] = -1;
        }
    }

    // VESC left communication timeout
    if (now - vesc_left.last_update_ms > VESC_TIMEOUT_MS) {
        Serial.println("SAFETY: Left VESC UART timeout");
        emergency_stop = true;
        if (now - error_last_sent[60] > CAN_ERROR_MESSAGE_MS){
            error_last_sent[60] = -1;
        }
    }

    // VESC right communication timeout
    if (now - vesc_right.last_update_ms > VESC_TIMEOUT_MS) {
        Serial.println("SAFETY: Right VESC UART timeout");
        emergency_stop = true;
        if (now - error_last_sent[61] > CAN_ERROR_MESSAGE_MS){
            error_last_sent[61] = -1;
        }
    }

    // VESC fault codes (0 = no fault)
    if (vesc_left.fault_code != 0) {
        Serial.printf("SAFETY: Left VESC fault code %d\n", vesc_left.fault_code);
        emergency_stop = true;
        if (now - error_last_sent[vesc_left.fault_code] > CAN_ERROR_MESSAGE_MS){
            error_last_sent[vesc_left.fault_code] = -1;
        }
    }
    if (vesc_right.fault_code != 0) {
        Serial.printf("SAFETY: Right VESC fault code %d\n", vesc_right.fault_code);
        emergency_stop = true;
        if (now - error_last_sent[vesc_right.fault_code + 29] > CAN_ERROR_MESSAGE_MS){
            error_last_sent[vesc_right.fault_code + 29] = -1;
        }
    }

    // Motor temperature limits
    if (vesc_left.temp_motor_C  > MOTOR_TEMP_LIMIT_C) {
        Serial.println("SAFETY: Motor over-temperature");
        emergency_stop = true;
        if (now - error_last_sent[62] > CAN_ERROR_MESSAGE_MS){
            error_last_sent[62] = -1;
        }
    }
    if (vesc_right.temp_motor_C > MOTOR_TEMP_LIMIT_C) {
        Serial.println("SAFETY: Motor over-temperature");
        emergency_stop = true;
        if (now - error_last_sent[63] > CAN_ERROR_MESSAGE_MS){
            error_last_sent[63] = -1;
        }
    }

    // FET temperature limits
    if (vesc_left.temp_fet_C  > FET_TEMP_LIMIT_C) {
        Serial.println("SAFETY: FET over-temperature");
        emergency_stop = true;
        if (now - error_last_sent[64] > CAN_ERROR_MESSAGE_MS){
            error_last_sent[64] = -1;
        }
    }
    if (vesc_right.temp_fet_C > FET_TEMP_LIMIT_C) {
        Serial.println("SAFETY: FET over-temperature");
        emergency_stop = true;
        if (now - error_last_sent[65] > CAN_ERROR_MESSAGE_MS){
            error_last_sent[65] = -1;
        }
    }
}

static void send_error(uint8_t error){
      // Prepare CAN message
  twai_message_t message = { 0 };
  message.identifier = 0x05;    // CAN ID
  message.extd = 0;              // Standard 11-bit ID
  message.data_length_code = 1;  // Data length

  // Pack data into CAN message
  message.data[0] = error;

  // Transmit message
  return  CANHelper::sendMessage(message);
}

// =============================================================
// SECTION 10 – TELEMETRY PRINT (debug output)
// =============================================================

static void printTelemetry() {
    static uint32_t last_print = 0;
    if (millis() - last_print < 250) return;    // 4 Hz – readable on serial monitor
    last_print = millis();

    const char* gear_str = (sensors.gear == FORWARD)  ? "FWD" :
                           (sensors.gear == REVERSE)  ? "REV" : "NEU";

    Serial.printf("[SENSORS] Steer: %+.1f° | Throttle: %.1f%% | Gear: %s | Speed: %.2f m/s | Yaw: %+.2f °/s\n",
        sensors.steering_deg, sensors.throttle_pct, gear_str,
        sensors.vehicle_speed_ms, sensors.yaw_rate_degs);

    Serial.printf("[L-VESC]  ERPM: %+7ld | I: %.1fA | Vin: %.1fV | Tfet: %.1f°C | Tmot: %.1f°C | Fault: %d\n",
        (long)vesc_left.erpm, vesc_left.motor_current_A, vesc_left.voltage_V,
        vesc_left.temp_fet_C, vesc_left.temp_motor_C, vesc_left.fault_code);

    Serial.printf("[R-VESC]  ERPM: %+7ld | I: %.1fA | Vin: %.1fV | Tfet: %.1f°C | Tmot: %.1f°C | Fault: %d\n",
        (long)vesc_right.erpm, vesc_right.motor_current_A, vesc_right.voltage_V,
        vesc_right.temp_fet_C, vesc_right.temp_motor_C, vesc_right.fault_code);

    if (emergency_stop) Serial.println("!!! EMERGENCY STOP ACTIVE – send 'r' to reset !!!");
}

// =============================================================
// SECTION 11 – SETUP
// =============================================================

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== E-Diff VESC 100/250 Controller booting ===");

    // Init left VESC UART (ESP32 UART1)
    VescLeft.begin(VESC_UART_BAUD, SERIAL_8N1, VESC_LEFT_RX, VESC_LEFT_TX);
    Serial.println("Left VESC UART initialised");

    // Init right VESC UART (ESP32 UART2)
    VescRight.begin(VESC_UART_BAUD, SERIAL_8N1, VESC_RIGHT_RX, VESC_RIGHT_TX);
    Serial.println("Right VESC UART initialised");

    // Init CAN bus
    CANHelper::setupCAN(CAN_RX_PIN, CAN_TX_PIN);

    // Seed timestamps so watchdog doesn't fire immediately on boot
    uint32_t now = millis();
    last_can_rx_ms          = now;
    vesc_left.last_update_ms  = now;
    vesc_right.last_update_ms = now;

    // Initialise sensor struct to safe defaults
    sensors.steering_deg     = 0.0f;
    sensors.throttle_pct     = 0.0f;
    sensors.gear             = NEUTRAL;
    sensors.vehicle_speed_ms = 0.0f;
    sensors.yaw_rate_degs    = 0.0f;

    emergency_stop = false;

    Serial.println("=== Boot complete – waiting for CAN messages ===");
    delay(500);
}

// =============================================================
// SECTION 12 – MAIN LOOP
// =============================================================
// Runs at LOOP_PERIOD_MS (default 10 ms / 100 Hz).
// Order of operations each cycle:
//  1. Process all pending CAN messages
//  2. Poll / parse VESC telemetry
//  3. Safety watchdog check
//  4. Calculate differential motor commands
//  5. Send current commands (or zero if e-stop active)
//  6. Print debug telemetry (rate-limited)

void loop() {
    static uint32_t loop_timer = 0;
    uint32_t now = millis();

    // Only execute control loop at the target rate
    if (now - loop_timer < LOOP_PERIOD_MS) return;
    loop_timer = now;

    // ── 1. Ingest CAN messages ───────────────────────────────
    processCAN();

    // ── 2. VESC telemetry ────────────────────────────────────
    updateVescTelemetry();

    // ── 3. Safety ────────────────────────────────────────────
    checkSafety();
    for (int i = 0; i < 66;i++){
        if (error_last_sent[i] == -1){
            send_error(i);
            error_last_sent[i] == millis();
        }
    }

    // ── 4. Calculate outputs ─────────────────────────────────
    MotorCommands cmd = calculateDifferential();

    // ── 5. Send commands ─────────────────────────────────────
    if (emergency_stop) {
        // Send 0A current to both VESCs to command them to stop
        vescSetCurrent(VescLeft,  0.0f);
        vescSetCurrent(VescRight, 0.0f);
    } else {
        vescSetCurrent(VescLeft,  cmd.left_A);
        vescSetCurrent(VescRight, cmd.right_A);
    }
    Serial.print("Left current: ");
    Serial.print(cmd.left_A);
    Serial.print(" Right current: ");
    Serial.println(cmd.right_A);

    // ── 6. Debug output ──────────────────────────────────────
    printTelemetry();

    // ── Serial commands (via USB monitor) ────────────────────
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'r' || c == 'R') {
            emergency_stop = false;
            // Re-seed VESC timestamps so watchdog has a grace period
            vesc_left.last_update_ms  = millis();
            vesc_right.last_update_ms = millis();
            Serial.println("Emergency stop RESET");
        }
        if (c == 's' || c == 'S') {
            emergency_stop = true;
            Serial.println("Emergency stop ACTIVATED by user");
        }
    }
}
