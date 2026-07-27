// ============================================================
//  CAN -> UART forwarder  (runs on the "hat" ESP32)
//  Reads the vehicle CAN bus and forwards the dashboard values
//  to the CrowPanel over UART.
//
//  Frame (newline terminated), 13 values:
//   $,speed,buttons,solar,gear,steer,soc,tmax,power,
//     motorT,ctrlT,mpptWarn,commsWarn,battFlags\n
//
//  Byte order: BIG-endian (Motorola) for IMU/steering/gear/MPPT/VESC,
//              LITTLE-endian (Intel) for the Prohelion BMS frames.
// ============================================================

#include <Arduino.h>
#include <driver/twai.h>
#include <CANHelper.h>
#include "config.h"

// NOTE: HardwareSerial LinkSerial(1); is declared in config.h.
//       If it is NOT in your config.h, uncomment the next line:
HardwareSerial LinkSerial(1);

// ---- Live state, updated from CAN ----
static float    g_speed_kph     = 0;
static uint8_t  g_buttons       = 0;
static float    g_solar_power_W = 0;
static uint8_t  g_gear          = 0;
static float    g_steering_deg  = 0;

static float    g_batt_soc      = -1;   // %   from 0x6F4
static float    g_batt_tmax     = -1;   // C   from 0x6F9 (max cell temp)
static float    g_batt_power    = -1;   // W   from 0x6FA (V*I)

static float    g_motor_temp    = -1;   // C   hottest motor (VESC Status 4)
static float    g_ctrl_temp     = -1;   // C   hottest MOSFET (VESC Status 4)
static uint8_t  g_mppt_warn     = 0;    // 1 = MPPT derating/fault
static uint8_t  g_comms_warn    = 0;    // 1 = a subsystem comms lost
static uint32_t g_batt_flags    = 0;    // 0x6FB status + 0x6FD extended flags

// per-VESC temp stores (index 0 = ctrl 1 / 0x1001, index 1 = ctrl 2 / 0x1002)
static float    g_mosfet[2]     = {-1, -1};
static float    g_motor[2]      = {-1, -1};

// last-seen timestamps for comms-timeout detection
static uint32_t g_seen_bms      = 0;
static uint32_t g_seen_vesc     = 0;
static uint32_t g_seen_mppt     = 0;

// ---- byte helpers ----
// big-endian / Motorola (IMU, steering, gear, MPPT, VESC)
static inline int16_t rd_i16(const uint8_t* d, int o) {
    return (int16_t)((d[o] << 8) | d[o + 1]);
}
// little-endian / Intel (Prohelion BMS)
static inline uint16_t rd_u16(const uint8_t* d, int o) {
    return (uint16_t)(d[o] | (d[o + 1] << 8));
}
static inline uint32_t rd_u32(const uint8_t* d, int o) {
    return (uint32_t)d[o] | ((uint32_t)d[o+1] << 8) |
           ((uint32_t)d[o+2] << 16) | ((uint32_t)d[o+3] << 24);
}
static inline int32_t rd_i32(const uint8_t* d, int o) {
    return (int32_t)rd_u32(d, o);
}
// little-endian float (Prohelion BMS)
static inline float rd_f32(const uint8_t* d, int o) {
    float f; memcpy(&f, d + o, 4); return f;
}

// recompute hottest motor / controller from the per-VESC stores
static void recompute_drivetrain_temps() {
    float ct = -1, mt = -1;
    for (int i = 0; i < 2; i++) {
        if (g_mosfet[i] > ct) ct = g_mosfet[i];
        if (g_motor[i]  > mt) mt = g_motor[i];
    }
    g_ctrl_temp  = ct;
    g_motor_temp = mt;
}

// Pull every queued CAN frame and update the live values
static void read_can() {
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        const uint8_t* d = msg.data;
        switch (msg.identifier) {

            case 0x031:  // IMU speed, INT16 @0.01 m/s, bytes 0-1 -> kph
                g_speed_kph = rd_i16(d, 0) * 0.01f * 3.6f;
                break;

            case 0x075:  // Steering-wheel button bitfield, byte 0
                g_buttons = d[0];
                break;

            case 0x011:  // Steering angle, INT16 @0.01 deg, bytes 0-1
                g_steering_deg = rd_i16(d, 0) * 0.01f;
                break;

            case 0x021:  // Accel/gear frame, gear switch in byte 2
                g_gear = d[2];
                break;

            // ---------- MPPT (single unit: power 0x200, status 0x201) ----------
            case 0x200: {   // Power: outV bytes 4-5 @0.01V, outI bytes 6-7 @0.0005A
                float outV = rd_i16(d, 4) * 0.01f;
                float outI = rd_i16(d, 6) * 0.0005f;
                g_solar_power_W = outV * outI;
                g_seen_mppt = millis();
                break;
            }
            case 0x201: {   // Status: byte0 mode (5=derating), byte1 fault
                uint8_t mode  = d[0];
                uint8_t fault = d[1];
                g_mppt_warn = (mode == 5 || fault != 0) ? 1 : 0;
                g_seen_mppt = millis();
                break;
            }

            // ---------- VESC Status 4 (Extended 0x1001 / 0x1002) ----------
            // MOSFET temp bytes 0-1, Motor temp bytes 2-3, INT16 @0.1C
            case 0x1001: {
                g_mosfet[0] = rd_i16(d, 0) * 0.1f;
                g_motor[0]  = rd_i16(d, 2) * 0.1f;
                recompute_drivetrain_temps();
                g_seen_vesc = millis();
                break;
            }
            case 0x1002: {
                g_mosfet[1] = rd_i16(d, 0) * 0.1f;
                g_motor[1]  = rd_i16(d, 2) * 0.1f;
                recompute_drivetrain_temps();
                g_seen_vesc = millis();
                break;
            }

            // ---------- Prohelion BMS (base 0x600, little-endian) ----------
            case 0x6F4: {   // Pack SoC: data_fp[1] already 0-100 %
                g_batt_soc = rd_f32(d, 4);
                g_seen_bms = millis();
                break;
            }
            case 0x6F9: {   // Min/Max cell temp: data_u16[1] = max, 1/10 C
                g_batt_tmax = rd_u16(d, 2) * 0.1f;
                g_seen_bms = millis();
                break;
            }
            case 0x6FA: {   // Pack V (u32 mV) & I (i32 mA) -> power W
                float volts = rd_u32(d, 0) / 1000.0f;
                float amps  = rd_i32(d, 4) / 1000.0f;
                g_batt_power = volts * amps;
                g_seen_bms = millis();
                break;
            }
            case 0x6FB: {   // Pack status flags in data_u8[4] (low byte)
                g_batt_flags = (g_batt_flags & 0xFFFFFF00) | d[4];
                g_seen_bms = millis();
                break;
            }
            case 0x6FD: {   // Extended status flags, full data_u32[0]
                g_batt_flags = rd_u32(d, 0);
                g_seen_bms = millis();
                break;
            }
        }
    }
}

// Update the comms-lost flag from the per-subsystem timeouts
static void update_comms_warn() {
    g_comms_warn = 0;
    uint32_t now = millis();
    if ((now - g_seen_bms)  > BMS_TIMEOUT_MS)  g_comms_warn += 1;
    if ((now - g_seen_vesc) > VESC_TIMEOUT_MS) g_comms_warn += 2;
    if ((now - g_seen_mppt) > MPPT_TIMEOUT_MS) g_comms_warn += 4;
}

// Emit one frame over the UART link
static void send_frame() {
    update_comms_warn();
    LinkSerial.printf("$,%d,%u,%.1f,%u,%.1f,%.0f,%.1f,%d,%.0f,%.0f,%u,%u,%lu\n",
        (int)lroundf(g_speed_kph), (unsigned)g_buttons, g_solar_power_W,
        (unsigned)g_gear, g_steering_deg, g_batt_soc, g_batt_tmax,
        (int)lroundf(g_batt_power),
        g_motor_temp, g_ctrl_temp,
        (unsigned)g_mppt_warn, (unsigned)g_comms_warn,
        (unsigned long)g_batt_flags);
}

void setup() {
    Serial.begin(115200);            // USB debug console
    delay(500);
    Serial.println("CAN->UART hat starting...");

    CANHelper::setupCAN(CAN_RX_PIN, CAN_TX_PIN);

    LinkSerial.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
    Serial.println("Link UART up.");
}

void loop() {
    read_can();                      // drain CAN -> update values

    static uint32_t last = 0;
    if (millis() - last >= SEND_INTERVAL_MS) {
        last = millis();
        send_frame();                // push one frame to the display

        // Echo to USB so you can watch the same frame on the serial monitor
        update_comms_warn();
        Serial.printf("$,%d,%u,%.1f,%u,%.1f,%.0f,%.1f,%d,%.0f,%.0f,%u,%u,%lu\n",
            (int)lroundf(g_speed_kph), (unsigned)g_buttons, g_solar_power_W,
            (unsigned)g_gear, g_steering_deg, g_batt_soc, g_batt_tmax,
            (int)lroundf(g_batt_power),
            g_motor_temp, g_ctrl_temp,
            (unsigned)g_mppt_warn, (unsigned)g_comms_warn,
            (unsigned long)g_batt_flags);
    }
}
