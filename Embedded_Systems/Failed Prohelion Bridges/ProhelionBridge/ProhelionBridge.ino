/*
 * =============================================================================
 *  Prohelion / Tritium CAN-Ethernet Bridge emulator for ESP32
 *  - hotspot / home WiFi edition with explicit MAC control -
 * =============================================================================
 *
 *  Use this version for testing on a phone hotspot or any WPA2-Personal
 *  network. For eduroam see prohelion_bridge_eduroam.ino instead.
 *
 *  Hardware:
 *    ESP32 dev board + SN65HVD230 transceiver
 *      ESP32 GPIO 5  (CAN_TX) ----> SN65HVD230 D    (TXD, pin 1)
 *      ESP32 GPIO 4  (CAN_RX) <---- SN65HVD230 R    (RXD, pin 4)
 *      ESP32 3.3V             ----> SN65HVD230 VCC  (pin 3)
 *      ESP32 GND              ----> SN65HVD230 GND  (pin 2)
 *      SN65HVD230 Rs (pin 8) tied to GND
 *      SN65HVD230 CANH/CANL  ----> BMU CAN-H / CAN-L
 *      120 ohm termination at both ends of the bus
 *
 *  Phone hotspot reminders:
 *    - Set hotspot to 2.4 GHz only (ESP32 cannot do 5 GHz)
 *      iPhone: Settings -> Personal Hotspot -> "Maximise Compatibility" ON
 *      Android: Hotspot settings -> AP band -> 2.4 GHz
 *    - Use WPA2 (or WPA2/WPA3), NOT WPA3-only
 *    - Keep the hotspot screen open while ESP32 connects, so it doesn't
 *      time out the radio
 *
 *  License: MIT
 * =============================================================================
 */

#include <WiFi.h>
#include <AsyncUDP.h>
#include "driver/twai.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_log.h"

// =============================================================================
//  USER CONFIGURATION  -  edit these for your setup
// =============================================================================

// WiFi credentials (your phone hotspot or any WPA2-Personal network)
static const char* WIFI_SSID     = "Iphone";
static const char* WIFI_PASSWORD = "password";

// -- Bridge MAC address ------------------------------------------------------
//
// Set USE_CUSTOM_MAC to 1 to force a specific MAC, or 0 to use the chip's
// own eFuse MAC (recommended on a real Espressif chip).
//
// If you set a custom MAC, the first byte MUST have:
//   - bit 0 = 0  (unicast, not multicast)
//   - bit 1 = 1  (locally-administered)
// So the first byte should look like xxxxxx10 in binary.
// Safe first-byte values: 0x02, 0x06, 0x0A, 0x0E, 0x12, 0x16, 0x1A, 0x1E, etc.
//
// The remaining 5 bytes can be any value you like - just make sure no other
// device on your network uses the same MAC.
//
#define USE_CUSTOM_MAC      0
static const uint8_t CUSTOM_MAC[6] = { 0x02, 0xB0, 0x1D, 0xFA, 0xCE, 0x01 };

// CAN bus configuration
#define CAN_BITRATE_KBPS    500
#define CAN_TX_GPIO         GPIO_NUM_5
#define CAN_RX_GPIO         GPIO_NUM_4

// Bridge bus number. Must be unique across Prohelion bridges on your LAN.
#define BRIDGE_BUS_NUMBER   13

// =============================================================================
//  Tritium Bridge Protocol Constants  -  do not edit
// =============================================================================

static const IPAddress TRITIUM_MCAST_IP(239, 255, 60, 60);
static const uint16_t  TRITIUM_MCAST_PORT = 4876;

static const uint8_t TRITIUM_V2_MAGIC[5] = { 0x54, 0x72, 0x69, 0xFD, 0xD6 };

#define FLAG_HEARTBEAT      0x80
#define FLAG_SETTINGS       0x40
#define FLAG_RTR            0x20
#define FLAG_EXTENDED_ID    0x10

#define HEARTBEAT_INTERVAL_MS   1000
#define PKT_LEN                 28

// =============================================================================
//  Globals
// =============================================================================

AsyncUDP udp;
uint8_t  bridgeMac[6];
uint32_t lastHeartbeatMs = 0;
uint32_t lastStatsMs     = 0;

volatile uint32_t canRxCount = 0;
volatile uint32_t canTxCount = 0;
volatile uint32_t udpRxCount = 0;
volatile uint32_t udpTxCount = 0;

// =============================================================================
//  MAC handling - must run BEFORE WiFi.mode() so the WiFi stack picks it up
// =============================================================================

static void setupBridgeMac() {
#if USE_CUSTOM_MAC
    memcpy(bridgeMac, CUSTOM_MAC, 6);

    // Validate the locally-administered bit so we don't accidentally
    // configure a globally-routable MAC that collides with real hardware.
    if ((bridgeMac[0] & 0x02) == 0) {
        Serial.println("WARN: CUSTOM_MAC first byte does not have the "
                       "locally-administered bit set (bit 1). "
                       "Use 0x02, 0x06, 0x0A, 0x0E, 0x12, etc.");
    }
    if ((bridgeMac[0] & 0x01) != 0) {
        Serial.println("ERROR: CUSTOM_MAC first byte has the multicast bit "
                       "set (bit 0). MAC must be unicast.");
    }

    esp_err_t err = esp_base_mac_addr_set(bridgeMac);
    if (err != ESP_OK) {
        Serial.printf("WARN: esp_base_mac_addr_set failed (0x%x)\n", err);
    }
    Serial.printf("Bridge MAC: %02X:%02X:%02X:%02X:%02X:%02X  (custom)\n",
                  bridgeMac[0], bridgeMac[1], bridgeMac[2],
                  bridgeMac[3], bridgeMac[4], bridgeMac[5]);
#else
    esp_read_mac(bridgeMac, ESP_MAC_WIFI_STA);

    bool macIsZero = true;
    for (int i = 0; i < 6; i++) {
        if (bridgeMac[i] != 0) { macIsZero = false; break; }
    }
    if (macIsZero) {
        Serial.println("WARN: eFuse MAC is empty, using fallback");
        bridgeMac[0] = 0x02; bridgeMac[1] = 0xB0; bridgeMac[2] = 0x1D;
        bridgeMac[3] = 0xFA; bridgeMac[4] = 0xCE; bridgeMac[5] = 0x01;
        esp_base_mac_addr_set(bridgeMac);
    }
    Serial.printf("Bridge MAC: %02X:%02X:%02X:%02X:%02X:%02X  (%s)\n",
                  bridgeMac[0], bridgeMac[1], bridgeMac[2],
                  bridgeMac[3], bridgeMac[4], bridgeMac[5],
                  macIsZero ? "fallback" : "eFuse");
#endif
}

// =============================================================================
//  Tritium packet builders
// =============================================================================

static void writeBusId(uint8_t* buf, uint16_t busNumber) {
    memcpy(buf, TRITIUM_V2_MAGIC, 5);
    buf[5] = (busNumber >> 8) & 0xFF;
    buf[6] =  busNumber       & 0xFF;
}

static void writeClientId(uint8_t* buf, const uint8_t* mac) {
    buf[0] = 0x00;
    memcpy(buf + 1, mac, 6);
}

static int buildCanPacket(uint8_t* buf,
                          uint32_t canId, uint8_t flags,
                          uint8_t length, const uint8_t* data) {
    writeBusId(buf, BRIDGE_BUS_NUMBER);
    writeClientId(buf + 7, bridgeMac);

    buf[14] = (canId >> 24) & 0xFF;
    buf[15] = (canId >> 16) & 0xFF;
    buf[16] = (canId >>  8) & 0xFF;
    buf[17] =  canId        & 0xFF;
    buf[18] = flags;
    buf[19] = length & 0x0F;

    memset(buf + 20, 0, 8);
    if (data && length > 0) {
        memcpy(buf + 20, data, length > 8 ? 8 : length);
    }
    return PKT_LEN;
}

static int buildHeartbeatPacket(uint8_t* buf) {
    uint8_t hbData[8];
    hbData[0] = (CAN_BITRATE_KBPS >> 8) & 0xFF;
    hbData[1] =  CAN_BITRATE_KBPS       & 0xFF;
    memcpy(hbData + 2, bridgeMac, 6);
    return buildCanPacket(buf, 0x000, FLAG_HEARTBEAT, 8, hbData);
}

// =============================================================================
//  CAN bus
// =============================================================================

static bool initCan() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);

    twai_timing_config_t t_config;
    switch (CAN_BITRATE_KBPS) {
        case 1000: t_config = TWAI_TIMING_CONFIG_1MBITS();   break;
        case 800:  t_config = TWAI_TIMING_CONFIG_800KBITS(); break;
        case 500:  t_config = TWAI_TIMING_CONFIG_500KBITS(); break;
        case 250:  t_config = TWAI_TIMING_CONFIG_250KBITS(); break;
        case 125:  t_config = TWAI_TIMING_CONFIG_125KBITS(); break;
        case 100:  t_config = TWAI_TIMING_CONFIG_100KBITS(); break;
        default:   t_config = TWAI_TIMING_CONFIG_500KBITS(); break;
    }

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("ERROR: twai_driver_install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("ERROR: twai_start failed");
        return false;
    }
    Serial.printf("CAN started: %d kbit/s, TX=GPIO%d RX=GPIO%d\n",
                  CAN_BITRATE_KBPS, CAN_TX_GPIO, CAN_RX_GPIO);
    return true;
}

static void pumpCanToUdp() {
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        canRxCount++;

        uint8_t flags = 0;
        if (msg.flags & TWAI_MSG_FLAG_EXTD) flags |= FLAG_EXTENDED_ID;
        if (msg.flags & TWAI_MSG_FLAG_RTR)  flags |= FLAG_RTR;

        uint8_t buf[PKT_LEN];
        int len = buildCanPacket(buf, msg.identifier, flags,
                                 msg.data_length_code, msg.data);
        if (udp.writeTo(buf, len, TRITIUM_MCAST_IP, TRITIUM_MCAST_PORT)) {
            udpTxCount++;
        }
    }
}

static void forwardToCan(uint32_t id, uint8_t flags,
                         uint8_t length, const uint8_t* data) {
    twai_message_t msg = {};
    msg.identifier = id;
    msg.data_length_code = (length > 8) ? 8 : length;
    if (flags & FLAG_EXTENDED_ID) msg.flags |= TWAI_MSG_FLAG_EXTD;
    if (flags & FLAG_RTR)         msg.flags |= TWAI_MSG_FLAG_RTR;
    memcpy(msg.data, data, msg.data_length_code);

    if (twai_transmit(&msg, pdMS_TO_TICKS(50)) == ESP_OK) {
        canTxCount++;
    }
}

// =============================================================================
//  UDP receive
// =============================================================================

static void onUdpPacket(AsyncUDPPacket& packet) {
    udpRxCount++;
    uint8_t* data = packet.data();
    size_t   len  = packet.length();

    if (len < PKT_LEN) return;
    if (memcmp(data, TRITIUM_V2_MAGIC, 5) != 0) return;

    uint16_t busNum = ((uint16_t)data[5] << 8) | data[6];
    if (busNum != BRIDGE_BUS_NUMBER) return;

    if (memcmp(data + 8, bridgeMac, 6) == 0) return;

    size_t offset = 14;
    while (offset + 14 <= len) {
        uint32_t canId = ((uint32_t)data[offset]   << 24) |
                         ((uint32_t)data[offset+1] << 16) |
                         ((uint32_t)data[offset+2] <<  8) |
                          (uint32_t)data[offset+3];
        uint8_t  flags  = data[offset + 4];
        uint8_t  length = data[offset + 5] & 0x0F;
        const uint8_t* frameData = data + offset + 6;

        if (!(flags & FLAG_HEARTBEAT) && !(flags & FLAG_SETTINGS)) {
            forwardToCan(canId, flags, length, frameData);
        }
        offset += 14;
    }
}

// =============================================================================
//  WiFi - WPA2 Personal (hotspot or home network)
// =============================================================================

static void connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    delay(100);

    Serial.printf("Connecting to SSID: %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("WiFi connecting");
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (millis() - start > 30000) {
            Serial.println("\nWiFi timeout - rebooting");
            ESP.restart();
        }
    }
    Serial.printf("\nWiFi connected: SSID=%s, IP=%s, RSSI=%d\n",
                  WIFI_SSID,
                  WiFi.localIP().toString().c_str(),
                  WiFi.RSSI());

    // Confirm the WiFi stack actually used our MAC.
    uint8_t actualMac[6];
    WiFi.macAddress(actualMac);
    Serial.printf("WiFi MAC in use: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  actualMac[0], actualMac[1], actualMac[2],
                  actualMac[3], actualMac[4], actualMac[5]);
}

// =============================================================================
//  Setup / loop
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(500);

    // Quiet the noisy ESP-IDF debug logs that produce garbled characters.
    esp_log_level_set("*", ESP_LOG_ERROR);

    Serial.println("\n=== Prohelion CAN-Ethernet Bridge Emulator (hotspot) ===");
    Serial.printf("Bus number: %u, Bitrate: %u kbit/s\n",
                  BRIDGE_BUS_NUMBER, CAN_BITRATE_KBPS);

    // Set MAC BEFORE any WiFi calls.
    setupBridgeMac();

    connectWiFi();

    if (!initCan()) {
        Serial.println("CAN init failed - halting");
        while (1) delay(1000);
    }

    if (udp.listenMulticast(TRITIUM_MCAST_IP, TRITIUM_MCAST_PORT)) {
        udp.onPacket(onUdpPacket);
        Serial.printf("UDP listening on %s:%u\n",
                      TRITIUM_MCAST_IP.toString().c_str(),
                      TRITIUM_MCAST_PORT);
    } else {
        Serial.println("ERROR: UDP multicast listen failed");
    }

    Serial.println("Bridge running.\n");
}

void loop() {
    pumpCanToUdp();

    uint32_t now = millis();

    if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeatMs = now;
        uint8_t buf[PKT_LEN];
        int len = buildHeartbeatPacket(buf);
        if (udp.writeTo(buf, len, TRITIUM_MCAST_IP, TRITIUM_MCAST_PORT)) {
            udpTxCount++;
        }
    }

    if (now - lastStatsMs >= 5000) {
        lastStatsMs = now;
        twai_status_info_t st;
        const char* state = "?";
        uint32_t busErrs = 0;
        if (twai_get_status_info(&st) == ESP_OK) {
            switch (st.state) {
                case TWAI_STATE_RUNNING:    state = "RUNNING";    break;
                case TWAI_STATE_STOPPED:    state = "STOPPED";    break;
                case TWAI_STATE_BUS_OFF:    state = "BUS_OFF";    break;
                case TWAI_STATE_RECOVERING: state = "RECOVERING"; break;
            }
            busErrs = st.bus_error_count;
        }
        Serial.printf("[%lus] CAN %s busErrs=%u  rx=%u tx=%u  UDP rx=%u tx=%u  RSSI=%d\n",
                      now / 1000, state, busErrs,
                      canRxCount, canTxCount, udpRxCount, udpTxCount,
                      WiFi.RSSI());
    }

    twai_status_info_t st;
    if (twai_get_status_info(&st) == ESP_OK) {
        if (st.state == TWAI_STATE_BUS_OFF) {
            twai_initiate_recovery();
        } else if (st.state == TWAI_STATE_STOPPED) {
            twai_start();
        }
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi dropped - reconnecting");
        connectWiFi();
    }

    delay(1);
}
