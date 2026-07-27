#include <Arduino.h>
#include <lvgl.h>
#include <ESP_Panel_Library.h>   // from ESP32_Display_Panel
#include "config.h"
#include "src/ui.h"
#include <HardwareSerial.h>

// LVGL draw buffer — use PSRAM for large buffer
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf1;
static lv_color_t *buf2;

ESP_Panel *panel = nullptr;

// ============================================================
//  CrowPanel side: receive the hat's UART frames.
//  Drop this into your dashboard sketch, replacing the
//  CAN-based get_data().  Frame format:
//     $,<speed_kph>,<buttons>,<solar_W>,<gear>,<steer_deg>\n
// ============================================================


HardwareSerial LinkSerial(1);     // UART1 peripheral

// ---- Dashboard state (read by your update_ui()) ----
static float    g_speed_kph     = 0;
static uint8_t  g_buttons       = 0;
static float    g_solar_power_W = 0;
static uint8_t  g_gear          = 0;
static float    g_steering_deg  = 0;

static float g_batt_soc  = -1;
static float g_batt_tmax = -1;
static float g_batt_power = -1;

static float    g_motor_temp = -1;   // C   (NEW - hat must forward)
static float    g_ctrl_temp  = -1;   // C   (NEW)
static uint8_t  g_mppt_warn  = 0;    // 1 = any MPPT derate/fault (NEW)
static uint8_t  g_comms_warn = 0;    // 1 = a subsystem comms lost (NEW)
static uint32_t g_batt_flags = 0;    // 0x6FB/0x6FD flags (NEW)

// Line assembly buffer
static char    s_line[64];
static uint8_t s_len = 0;

// Call once in setup(), after your panel/LVGL init
void link_begin() {
    LinkSerial.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, /*TX=*/-1);
}

static void parse_line(char* line) {
    if (line[0] != '$') return;

    int      speed, power;
    unsigned buttons, gear, mpptw, commsw;
    float    solar, steer, soc, tmax, motorT, ctrlT;
    unsigned long flags;

    int n = sscanf(line, "$,%d,%u,%f,%u,%f,%f,%f,%d,%f,%f,%u,%u,%lu",
                   &speed, &buttons, &solar, &gear, &steer, &soc, &tmax,
                   &power, &motorT, &ctrlT, &mpptw, &commsw, &flags);
    if (n == 13) {
        g_speed_kph=(float)speed; g_buttons=(uint8_t)buttons;
        g_solar_power_W=solar; g_gear=(uint8_t)gear; g_steering_deg=steer;
        g_batt_soc=soc; g_batt_tmax=tmax; g_batt_power=(float)power;
        g_motor_temp=motorT; g_ctrl_temp=ctrlT;
        g_mppt_warn=(uint8_t)mpptw; g_comms_warn=(uint8_t)commsw;
        g_batt_flags=(uint32_t)flags;
    }
}

// Non-blocking: consume whatever bytes are waiting, parse on newline
void get_data() {
    while (LinkSerial.available()) {
        char c = (char)LinkSerial.read();

        if (c == '\n' || c == '\r') {
            if (s_len > 0) {
                s_line[s_len] = '\0';
                parse_line(s_line);
                s_len = 0;
            }
        } else if (s_len < sizeof(s_line) - 1) {
            s_line[s_len++] = c;
        } else {
            // overflow (garbage / desync) -> drop and resync on next '\n'
            s_len = 0;
        }
    }
}

/* LVGL display flush callback */
void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
    ESP_PanelLcd *lcd = (ESP_PanelLcd *)drv->user_data;
    lcd->drawBitmap(area->x1, area->y1,
                    area->x2 - area->x1 + 1,
                    area->y2 - area->y1 + 1,
                    (uint8_t *)color_p);
    lv_disp_flush_ready(drv);
}

static void draw_tick_ring() {
    // Hide the rotated SquareLine ticks (they don't render on hardware)
    lv_obj_t* old[] = {ui_Tick11,ui_Tick12,ui_Tick13,ui_Tick14,ui_Tick15,
                       ui_Tick16,ui_Tick17,ui_Tick18,ui_Tick19,ui_Tick20,ui_Tick21};
    for (lv_obj_t* t : old) if (t) lv_obj_add_flag(t, LV_OBJ_FLAG_HIDDEN);

    const int   N        = 9;     // number of ticks
    const float CX        = 200;   // centre of the 400x400 Speedometer
    const float CY        = 200;
    const float R_OUT     = 190;   // outer radius of tick
    const float R_IN      = 150;   // inner radius (R_OUT - R_IN = tick length)
    const float START_DEG = 120;   // sweep start
    const float END_DEG   = 420;   // sweep end (120->420 covers left, top, right)

    // Two point arrays now: one for shadow lines, one for the bright ticks.
    // Must persist (lv_line stores the pointer).
    static lv_point_t pts[11][2];
    static lv_point_t spts[11][2];

    const float SH_DX = 2.0f;   // shadow offset (down-right)
    const float SH_DY = 2.0f;

    for (int i = 0; i < N; i++) {
        float frac = (N > 1) ? (float)i / (N - 1) : 0;
        float a    = (START_DEG + frac * (END_DEG - START_DEG)) * 3.14159265f / 180.0f;
        float c = cosf(a), s = sinf(a);

        // bright tick endpoints
        pts[i][0].x = (lv_coord_t)(CX + R_IN  * c);
        pts[i][0].y = (lv_coord_t)(CY + R_IN  * s);
        pts[i][1].x = (lv_coord_t)(CX + R_OUT * c);
        pts[i][1].y = (lv_coord_t)(CY + R_OUT * s);

        // shadow endpoints = same, offset slightly
        spts[i][0].x = pts[i][0].x + (lv_coord_t)SH_DX;
        spts[i][0].y = pts[i][0].y + (lv_coord_t)SH_DY;
        spts[i][1].x = pts[i][1].x + (lv_coord_t)SH_DX;
        spts[i][1].y = pts[i][1].y + (lv_coord_t)SH_DY;

        // --- shadow line (drawn first, sits underneath) ---
        lv_obj_t* sh = lv_line_create(ui_Speedometer);
        lv_line_set_points(sh, spts[i], 2);
        lv_obj_set_style_line_width(sh, 9, LV_PART_MAIN | LV_STATE_DEFAULT);   // a touch wider
        lv_obj_set_style_line_color(sh, lv_color_hex(0x0A0820), LV_PART_MAIN | LV_STATE_DEFAULT); // dark, near-bg
        lv_obj_set_style_line_opa(sh, 140, LV_PART_MAIN | LV_STATE_DEFAULT);   // semi-transparent
        lv_obj_set_style_line_rounded(sh, true, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_move_background(sh);   // behind the arc
        

        // --- bright tick (drawn on top of its shadow) ---
        lv_obj_t* line = lv_line_create(ui_Speedometer);
        lv_line_set_points(line, pts[i], 2);
        lv_obj_set_style_line_width(line, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_line_color(line, lv_color_hex(0xC8CCD8), LV_PART_MAIN | LV_STATE_DEFAULT); // off-white
        lv_obj_set_style_line_rounded(line, true, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_move_background(line);   // behind the arc, but above its shadow
    }

    // Ensure the speed arc sits in front of all ticks
    lv_obj_move_foreground(ui_SpeedSlider1);
}

static bool hyst_high(float v, float on, float clr, bool prev){
    if (v >= on)  return true;
    if (v <= clr) return false;
    return prev;
}
static bool hyst_low(float v, float on, float clr, bool prev){
    if (v <= on)  return true;
    if (v >= clr) return false;
    return prev;
}

static void update_ui() {
    char buf[16];

    // --- Speed ---
    int speed = (int)lroundf(g_speed_kph);
    if (speed < 0) speed = 0;
    snprintf(buf, sizeof(buf), "%d", speed);
    lv_label_set_text(ui_Speed1, buf);
    lv_arc_set_value(ui_SpeedSlider1, speed);

    // --- Steering angle (signed, ±0 at zero) ---
    int ang = (int)lroundf(g_steering_deg);
    if (ang == 0) {
        snprintf(buf, sizeof(buf), "±0\u00B0");   // ±0°
    } else {
        snprintf(buf, sizeof(buf), "%+d\u00B0", ang);  // +5°, -12°
    }
    lv_label_set_text(ui_SteeringAngleLabel, buf);

    // --- Gear ---
    const char* gear_txt = "N";
    switch (g_gear) {
        case 1: gear_txt = "D";   break;
        case 2: gear_txt = "R";   break;
        case 3: gear_txt = "Err"; break;
        default: gear_txt = "N";  break;
    }
    lv_label_set_text(ui_DrivingStateLabel, gear_txt);

    // --- Solar input power ---
    snprintf(buf, sizeof(buf), "+%dW", (int)lroundf(g_solar_power_W));
    lv_label_set_text(ui_PowerIn, buf);

    // --- Indicators: blink while active ---
    bool hazard = g_buttons & 128;
    bool right  = g_buttons & 64;
    bool left   = g_buttons & 32;
    bool full = g_buttons & 16;

    bool want_right = right || hazard;
    bool want_left  = left  || hazard;

    // Blink phase: true for 500ms, false for 500ms, repeating
    int blink_pos = (millis() / FLASH_SPEED) % 3;

    // A lamp is lit only when it's requested AND we're in the "on" half of the blink
    lv_obj_set_style_opa(ui_FirstPartLitRightIndicator, (want_right && blink_pos == 0) ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_FullLitRightIndicator,  (want_right  && blink_pos == 1) ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_SecondPartLitRightIndicator, (want_right && blink_pos == 2) ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_FirstPartLitLeftIndicator,  (want_left  && blink_pos == 0) ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_FullLitLeftIndicator, (want_left && blink_pos == 1) ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_SecondPartLitLeftIndicator,  (want_left  && blink_pos == 2) ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);


    // Warning/hazard triangle: blink with hazards, steady-off otherwise
    lv_obj_set_style_opa(ui_LitWarningLight, hazard ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Warning/hazard triangle: blink with hazards, steady-off otherwise
    lv_obj_set_style_opa(ui_LitFullBeam, full ? 255 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    // --- Battery % ---
    if (g_batt_soc < 0) {
        lv_label_set_text(ui_BattPerc2, "N/A");
        lv_slider_set_value(ui_BattPercSlider3, 0, LV_ANIM_OFF);
    } else {
        int soc = (int)lroundf(g_batt_soc);
        if (soc < 0) soc = 0; if (soc > 100) soc = 100;
        lv_slider_set_value(ui_BattPercSlider3, soc, LV_ANIM_OFF);
        snprintf(buf, sizeof(buf), "%d%%", soc);
        lv_label_set_text(ui_BattPerc2, buf);
    }

    // --- Battery Power ---
    if (g_batt_power < 0) {
        lv_label_set_text(ui_PowerOut, "N/A");
    } else {
        snprintf(buf, sizeof(buf), "%dW", (int)lroundf(g_batt_power)); 
        lv_label_set_text(ui_PowerOut, buf);
    }

    // ===== Driver-awareness warnings =====
    static bool w_batt=false, w_temp=false, w_motor=false,
                w_ctrl=false, w_solar=false, w_comms=false;

    if (g_batt_soc  >= 0) w_batt  = hyst_low (g_batt_soc,  BATT_LOW_ON,  BATT_LOW_CLEAR,  w_batt);
    if (g_batt_tmax >= 0) w_temp  = hyst_high(g_batt_tmax, BATT_TEMP_ON, BATT_TEMP_CLEAR, w_temp);
    if (g_motor_temp>= 0) w_motor = hyst_high(g_motor_temp,MOTOR_TEMP_ON,MOTOR_TEMP_CLEAR,w_motor);
    if (g_ctrl_temp >= 0) w_ctrl  = hyst_high(g_ctrl_temp, CTRL_TEMP_ON, CTRL_TEMP_CLEAR, w_ctrl);

    w_solar = (g_mppt_warn != 0);
    bool batt_data_bad = g_batt_flags & (0x08 | 0x10) || false;   // untrusted | lost CMU
    w_comms = (g_comms_warn != 0) || batt_data_bad;

    // Drive the six strip overlays (amber overlay = image opacity)
    lv_obj_set_style_opa(ui_StatBattWarn,  w_batt ?255:0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_StatTempWarn,  w_temp ?255:0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_StatMotorWarn, w_motor?255:0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_StatCtrlWarn,  w_ctrl ?255:0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_StatSolarWarn, w_solar?255:0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_StatCommsWarn, w_comms?255:0, LV_PART_MAIN|LV_STATE_DEFAULT);

    // Dynamic warning text: cycle through whatever is active, hide if none
    const char* msgs[8]; int nmsg = 0;
    if (w_temp)  msgs[nmsg++] = "Battery temp high";
    if (w_batt)  msgs[nmsg++] = "Battery low";
    if (w_motor) msgs[nmsg++] = "Motor temp high";
    if (w_ctrl)  msgs[nmsg++] = "Controller temp high";
    if (w_solar) msgs[nmsg++] = "Solar derating";
    if ((g_comms_warn & 1) || batt_data_bad) msgs[nmsg++] = "Battery data unreliable";
    if (g_comms_warn & 2) msgs[nmsg++] = "VESC data unreliable";
    if (g_comms_warn & 4) msgs[nmsg++] = "MPPT data unreliable";

    if (nmsg == 0) {
        lv_obj_set_style_opa(ui_WarningText, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    } else {
        int idx = (millis() / WARN_TEXT_CYCLE_MS) % nmsg;
        lv_label_set_text(ui_WarningText, msgs[idx]);
        lv_obj_set_style_opa(ui_WarningText, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Serial monitor running...");
    Serial.flush();

    // Init panel. The board-support library still initialises the touch
    // controller at the hardware level, but nothing reads from it, so the
    // display behaves as a display-only gauge cluster.
    panel = new ESP_Panel();
    panel->init();
    Serial.println("Panel initialised...");
    Serial.flush();
    panel->begin();
    Serial.println("Panel Begun...");
    Serial.flush();

    // LVGL init
    lv_init();
    Serial.println("LVGL initialised");
    Serial.flush();

    // Allocate draw buffers in PSRAM
    // Smaller buffers to reduce memory usage
    const uint32_t buf_size = SCREEN_WIDTH * 20; // Reduced from 40 to 20

    buf1 = (lv_color_t *)heap_caps_malloc(
        buf_size * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    buf2 = (lv_color_t *)heap_caps_malloc(
        buf_size * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, buf_size);
    Serial.println("Buffers allocated PSRAM");
    Serial.flush();

    // Register display
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res  = SCREEN_WIDTH;
    disp_drv.ver_res  = SCREEN_HEIGHT;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.user_data = panel->getLcd();
    lv_disp_drv_register(&disp_drv);
    Serial.println("Display registered...");
    Serial.flush();

    // (Touch input device intentionally not registered — display-only.)

    // Init your SquareLine UI
    ui_init();
    lv_obj_set_style_pad_all(ui_Speedometer, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    draw_tick_ring();
    Serial.println("UI initialised...");
    Serial.flush();

    link_begin();
    Serial.println("link UART started on IO38");
}

void loop() {
    get_data();

    static uint32_t last = 0;
    if (millis() - last >= 100) {     // refresh UI at 10 Hz
        last = millis();
        update_ui();
    }

    lv_timer_handler();
    delay(5);
}
