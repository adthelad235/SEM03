// =============================================================
// CAN_Tester.ino
// Sends all four CAN messages at 50 Hz for testing the
// E-Diff VESC controller.
//
// Wiring:
//   GPIO 5 → CAN transceiver TX
//   GPIO 4 → CAN transceiver RX
//
// Serial monitor: 115200 baud
// Commands (type number, press enter):
//   0  STOP          – neutral, zero throttle
//   1  Straight      – 25% throttle forward, no steering
//   2  Right turn    – 50% throttle forward, 10° right
//   3  Left turn     – 50% throttle forward, 10° left
//   4  Sharp right   – 75% throttle forward, 25° right
//   5  Sharp left    – 75% throttle forward, 25° left
//   6  Reverse       – 25% throttle reverse, straight
//   7  Oversteer     – 50% throttle, 10° right, yaw too high
// =============================================================

#include <driver/twai.h>

// ── CAN pins ──────────────────────────────────────────────────
#define CAN_TX_PIN  GPIO_NUM_5
#define CAN_RX_PIN  GPIO_NUM_4

// ── CAN IDs (must match config.h on the controller) ───────────
#define CAN_ID_THROTTLE  0x021
#define CAN_ID_STEERING  0x011
#define CAN_ID_SPEED     0x030
#define CAN_ID_IMU       0x031

// ── Broadcast interval ────────────────────────────────────────
#define TX_INTERVAL_MS  20      // 50 Hz

// =============================================================
// SCENARIO DEFINITIONS
//
// throttle_adc  : raw ADC value 0–4095
// gear          : 0=neutral  1=forward  2=reverse
// steering_cdeg : steering degrees * 100  (e.g. 1000 = 10.00°)
// speed_cms     : vehicle speed in cm/s  (signed INT16)
// yaw_cdeg      : yaw rate (deg/s) * 100  (signed INT16)
// =============================================================
struct Scenario {
    const char* name;
    int16_t  throttle_adc;
    uint8_t  gear;
    int16_t  steering_cdeg;
    int16_t  speed_cms;
    int16_t  yaw_cdeg;
};

const Scenario scenarios[] = {

    // 0 ── STOP
    { "STOP – neutral, zero throttle",
       0,    0,     0,    0,     0 },

    // 1 ── Straight forward 25%
    // Zero steering, zero yaw – clean baseline to confirm VESC gets equal currents
    { "Straight – 25% throttle, forward",
       1024, 1,     0,  150,     0 },

    // 2 ── Gentle right turn 50%, 10°
    // Expected yaw ≈ (3.0 m/s * tan10°) / 1.4 m track ≈ 8 deg/s → 800 cdeg
    { "Right turn – 50% throttle, 10 deg right",
       2048, 1,  1000,  300,   800 },

    // 3 ── Gentle left turn 50%, 10°
    { "Left turn – 50% throttle, 10 deg left",
       2048, 1, -1000,  300,  -800 },

    // 4 ── Sharp right 75%, 25°
    // Expected yaw ≈ (4.0 m/s * tan25°) / 1.4 ≈ 8.4 deg/s → 840 cdeg
    { "Sharp right – 75% throttle, 25 deg right",
       3072, 1,  2500,  400,   840 },

    // 5 ── Sharp left 75%, 25°
    { "Sharp left – 75% throttle, 25 deg left",
       3072, 1, -2500,  400,  -840 },

    // 6 ── Reverse straight 25%
    { "Reverse – 25% throttle, straight",
       1024, 2,     0, -150,     0 },

    // 7 ── Oversteer: steering 10° right but yaw 20 deg/s (way over expected 8)
    // Controller should reduce left (outer) wheel current to fight the spin
    { "Oversteer – 50% throttle, 10 deg right, yaw 20 deg/s",
       2048, 1,  1000,  300,  2000 },
};

const int NUM_SCENARIOS = sizeof(scenarios) / sizeof(scenarios[0]);

// =============================================================
// GLOBALS
// =============================================================
int      active  = 0;
uint32_t last_tx = 0;

// =============================================================
// CAN TRANSMIT HELPER
// =============================================================
static void sendCAN(uint32_t id, const uint8_t* data, uint8_t len) {
    twai_message_t msg = {};
    msg.identifier       = id;
    msg.data_length_code = len;
    memcpy(msg.data, data, len);
    if (twai_transmit(&msg, pdMS_TO_TICKS(5)) != ESP_OK) {
        Serial.printf("  [WARN] TX failed for ID 0x%03X\n", id);
    }
}

// =============================================================
// MESSAGE BUILDERS
// All values packed big-endian to match the controller decoders
// =============================================================

// 0x021  byte 0-1: throttle ADC INT16   byte 2: gear
static void txThrottle(int16_t adc, uint8_t gear) {
    uint8_t d[3];
    d[0] = (adc >> 8) & 0xFF;
    d[1] =  adc       & 0xFF;
    d[2] = gear;
    sendCAN(CAN_ID_THROTTLE, d, 3);
}

// 0x011  byte 0-1: steering degrees * 100, INT16 signed
static void txSteering(int16_t cdeg) {
    uint8_t d[2];
    d[0] = (cdeg >> 8) & 0xFF;
    d[1] =  cdeg       & 0xFF;
    sendCAN(CAN_ID_STEERING, d, 2);
}

// 0x030  byte 0-1: speed cm/s, INT16 signed
static void txSpeed(int16_t cms) {
    uint8_t d[2];
    d[0] = (cms >> 8) & 0xFF;
    d[1] =  cms       & 0xFF;
    sendCAN(CAN_ID_SPEED, d, 2);
}

// 0x031  byte 0-1: yaw rate deg/s * 100, INT16 signed
static void txIMU(int16_t cdeg) {
    uint8_t d[2];
    d[0] = (cdeg >> 8) & 0xFF;
    d[1] =  cdeg       & 0xFF;
    sendCAN(CAN_ID_IMU, d, 2);
}

// =============================================================
// PRINT HELPERS
// =============================================================
static void printScenario(int idx) {
    const Scenario& s = scenarios[idx];
    Serial.println("\n-----------------------------------------");
    Serial.printf("Active: [%d] %s\n", idx, s.name);
    Serial.printf("  Throttle : %d ADC  (%.1f%%)\n",
        s.throttle_adc, s.throttle_adc / 4095.0f * 100.0f);
    Serial.printf("  Gear     : %s\n",
        s.gear == 1 ? "FORWARD" : s.gear == 2 ? "REVERSE" : "NEUTRAL");
    Serial.printf("  Steering : %.2f deg\n",   s.steering_cdeg / 100.0f);
    Serial.printf("  Speed    : %.2f m/s\n",   s.speed_cms     / 100.0f);
    Serial.printf("  Yaw rate : %.2f deg/s\n", s.yaw_cdeg      / 100.0f);
    Serial.println("-----------------------------------------");
}

static void printMenu() {
    Serial.println("\n====== CAN Tester – type a number + enter ======");
    for (int i = 0; i < NUM_SCENARIOS; i++) {
        Serial.printf("  %d  %s\n", i, scenarios[i].name);
    }
    Serial.println("================================================\n");
}

// =============================================================
// SETUP
// =============================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== CAN Tester booting ===");

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());
    Serial.println("CAN started at 500 kbit/s");

    printMenu();
    printScenario(active);
}

// =============================================================
// MAIN LOOP
// =============================================================
void loop() {

    // ── Serial input: switch scenario ────────────────────────
    if (Serial.available()) {
        char c = Serial.read();
        if (c >= '0' && c <= '9') {
            int choice = c - '0';
            if (choice < NUM_SCENARIOS) {
                active = choice;
                printScenario(active);
            } else {
                Serial.printf("No scenario %d – valid range 0-%d\n",
                    choice, NUM_SCENARIOS - 1);
            }
        }
    }

    // ── Transmit all four messages at 50 Hz ──────────────────
    uint32_t now = millis();
    if (now - last_tx >= TX_INTERVAL_MS) {
        last_tx = now;
        const Scenario& s = scenarios[active];
        txThrottle(s.throttle_adc, s.gear);
        txSteering(s.steering_cdeg);
        txSpeed(s.speed_cms);
        txIMU(s.yaw_cdeg);
    }
}
