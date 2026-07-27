// =============================================================
// config.h  –  All pin definitions, CAN IDs, and tuning values
// =============================================================

// ── UART pins for the two VESC 100/250 controllers ──────────
#define VESC_LEFT_TX    17
#define VESC_LEFT_RX    16
#define VESC_RIGHT_TX   26
#define VESC_RIGHT_RX   25

// ── TWAI (CAN) pins ──────────────────────────────────────────
#define CAN_TX_PIN      GPIO_NUM_5
#define CAN_RX_PIN      GPIO_NUM_4

// ── CAN message IDs (incoming) ───────────────────────────────
// Throttle + gear:  bytes 0-1 = raw ADC INT16, byte 2 = gear (0=N, 1=F, 2=R)
#define CAN_ID_THROTTLE     0x021
// Steering angle:   bytes 0-1 = INT16 angle * 100 (e.g. 3000 = 30.00 deg)
#define CAN_ID_STEERING     0x011
// Vehicle speed:    bytes 0-1 = INT16 speed in cm/s  (signed, negative = reverse)
// IMU yaw rate:     bytes 2-3 = INT16 yaw rate * 100  (deg/s * 100, signed)
#define CAN_ID_IMU          0x031

// ── UART baud rate ───────────────────────────────────────────
// VESC 100/250 default is 115200; change here and in VESC Tool if needed
#define VESC_UART_BAUD      115200

// ── Motor geometry ───────────────────────────────────────────
// Motor pole PAIRS (not total poles).  Trampa 100/250 motors vary –
// measure or check your motor spec sheet.
// ERPM = RPM * pole_pairs.  Used to convert ERPM → wheel RPM.
const int   MOTOR_POLE_PAIRS    = 7;       // 14-pole motor → 7 pairs

// Wheel and vehicle dimensions
const float WHEEL_DIAMETER_M    = 0.60f;   // metres – measure your tyre OD
const float WHEEL_TRACK_M       = 1.40f;   // metres – centre-to-centre of tyres

// ── Throttle ADC calibration ─────────────────────────────────
const int   THROTTLE_ADC_MIN    = 0;
const int   THROTTLE_ADC_MAX    = 4095;    // 12-bit ADC

// ── Steering calibration ─────────────────────────────────────
// Maximum physical steering angle in degrees (set to your lock-to-lock)
const float STEERING_MAX_DEG    = 30.0f;

// ── Motor current limits ─────────────────────────────────────
const float MAX_MOTOR_CURRENT_A = 200.0f;   // Peak amps per motor

// ── E-differential tuning ────────────────────────────────────
// How much speed difference is applied per degree of steering.
// Increase for sharper differential effect.
const float ACKERMANN_GAIN      = 0.15f;

// ── Torque-vectoring (yaw correction) ───────────────────────
// Scales how aggressively the controller corrects yaw error.
// Start very low (0.1 – 0.3) and increase during testing.
const float YAW_CORRECTION_GAIN = 0.2f;

// ── Traction control ─────────────────────────────────────────
// ERPM difference above the expected value that indicates wheel slip.
// Tune based on motor pole pairs and wheel size.
const float SLIP_THRESHOLD_ERPM = 350.0f;  // ~50 RPM on a 7-pair motor

// Current multiplier applied when slip is detected (0.0 – 1.0).
const float TRACTION_CTRL_FACTOR = 0.70f;

// ── Safety timeouts ──────────────────────────────────────────
// If no CAN messages have arrived within this period → emergency stop.
const uint32_t CAN_TIMEOUT_MS       = 200;
// If no VESC telemetry reply arrives within this period → emergency stop.
const uint32_t VESC_TIMEOUT_MS      = 1000000;
// Motor temperature limit (°C) – above this triggers emergency stop.
const float    MOTOR_TEMP_LIMIT_C   = 80.0f;
// FET temperature limit (°C)
const float    FET_TEMP_LIMIT_C     = 80.0f;

// ── Control loop rate ────────────────────────────────────────
// Main loop target period in milliseconds (100 Hz = 10 ms).
const uint32_t LOOP_PERIOD_MS       = 10;

// ── VESC telemetry request rate ─────────────────────────────
// How often to request COMM_GET_VALUES from each VESC (milliseconds).
const uint32_t VESC_TELEMETRY_RATE_MS = 20;  // 50 Hz
// How often error messages get sent
const uint32_t CAN_ERROR_MESSAGE_MS = 2000 // 0.5 Hz
