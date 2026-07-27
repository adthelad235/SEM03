// ---- Pins: set these to whatever is free on YOUR hat ----
#define CAN_RX_PIN GPIO_NUM_19
#define CAN_TX_PIN GPIO_NUM_21

// UART to the CrowPanel. UART1 on any two free pins.
// Only TX is actually needed (one-way to the display).
#define LINK_TX_PIN  GPIO_NUM_5    // -> CrowPanel IO38 (its RX)
#define LINK_RX_PIN  GPIO_NUM_18    // unused, but HardwareSerial wants a pin
#define LINK_BAUD    115200

#define SEND_INTERVAL_MS  50        // 20 Hz frame rate to the display

// ---- Comms-timeout windows (sized to each subsystem's frame rate) ----
#define BMS_TIMEOUT_MS    500    // BMS chatters at up to 10Hz (0x6FA/0x6F8)
#define VESC_TIMEOUT_MS   1000   // VESC Status ~4Hz -> ~4 missed frames
#define MPPT_TIMEOUT_MS   3000   // MPPT 1-2Hz -> ~3 missed frames

