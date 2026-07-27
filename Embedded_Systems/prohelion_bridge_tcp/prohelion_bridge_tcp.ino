/*
 * =============================================================================
 *  Prohelion / Tritium CAN-Ethernet Bridge emulator for ESP32
 *  - hotspot edition with TCP support -
 * =============================================================================
 *
 *  This version adds TCP server support alongside UDP. Use this when running
 *  on networks that block multicast (notably iPhone Personal Hotspot, which
 *  doesn't reliably forward multicast packets between connected clients).
 *
 *  Architecture:
 *    - UDP heartbeat continues to be sent (helps Profinity AutoDiscovery)
 *    - TCP server listens on port 4876 for Profinity connections
 *    - When Profinity connects via TCP, all CAN frames flow over that
 *      connection in both directions (Profinity sets fwd range to all)
 *    - If no TCP client connected, falls back to UDP multicast as before
 *
 *  Profinity-side configuration (REQUIRED — see README in chat):
 *    1. AutoDiscovery may or may not pick up the bridge. If it doesn't,
 *       manually add a Prohelion CAN-Ethernet Bridge V2.
 *    2. Set the bridge's connection mode to TCP.
 *    3. Specify the ESP32's IP address (shown in serial monitor at boot).
 *    4. Bus number must match BRIDGE_BUS_NUMBER below.
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
//  USER CONFIGURATION
// =============================================================================

static const char* WIFI_SSID     = "Iphone";
static const char* WIFI_PASSWORD = "password";

#define USE_CUSTOM_MAC      1
static const uint8_t CUSTOM_MAC[6] = { 0x02, 0xB0, 0x1D, 0xFA, 0xCE, 0x01 };

#define CAN_BITRATE_KBPS    500
#define CAN_TX_GPIO         GPIO_NUM_5
#define CAN_RX_GPIO         GPIO_NUM_4

#define BRIDGE_BUS_NUMBER   13

// =============================================================================
//  Tritium Bridge Protocol Constants
// =============================================================================

static const IPAddress TRITIUM_MCAST_IP(239, 255, 60, 60);
static const uint16_t  TRITIUM_PORT = 4876;

static const uint8_t TRITIUM_V2_MAGIC[5] = { 0x54, 0x72, 0x69, 0xFD, 0xD6 };

#define FLAG_HEARTBEAT      0x80
#define FLAG_SETTINGS       0x40
#define FLAG_RTR            0x20
#define FLAG_EXTENDED_ID    0x10

#define HEARTBEAT_INTERVAL_MS   1000

// UDP single-frame packet:
//   8 (bus_id, 64-bit aligned with leading zero byte) +
//   8 (client_id, same) +
//   4 (CAN identifier) + 1 (flags) + 1 (length) + 8 (data)
// = 30 bytes
#define UDP_PKT_LEN     30

// TCP initial header: 4 (fwd_id) + 4 (fwd_range) + 8 (bus_id) + 8 (client_id) = 24 bytes
#define TCP_HEADER_LEN  24
// TCP per-frame: 4 (ID) + 1 (flags) + 1 (length) + 8 (data) = 14 bytes
#define TCP_FRAME_LEN   14

// =============================================================================
//  Globals
// =============================================================================

AsyncUDP   udp;
WiFiServer tcpServer(TRITIUM_PORT);
WiFiClient tcpClient;

uint8_t  bridgeMac[6];
uint32_t lastHeartbeatMs = 0;
uint32_t lastStatsMs     = 0;

// TCP receive state
bool     tcpHeaderDone   = false;
uint8_t  tcpRxBuf[256];
size_t   tcpRxLen        = 0;
uint32_t tcpFwdId        = 0;
uint32_t tcpFwdRange     = 0;

volatile uint32_t canRxCount = 0;
volatile uint32_t canTxCount = 0;
volatile uint32_t udpRxCount = 0;
volatile uint32_t udpTxCount = 0;
volatile uint32_t tcpRxCount = 0;
volatile uint32_t tcpTxCount = 0;

// =============================================================================
//  MAC handling
// =============================================================================

static void setupBridgeMac() {
#if USE_CUSTOM_MAC
    memcpy(bridgeMac, CUSTOM_MAC, 6);
    esp_base_mac_addr_set(bridgeMac);
    Serial.printf("Bridge MAC: %02X:%02X:%02X:%02X:%02X:%02X  (custom)\n",
                  bridgeMac[0], bridgeMac[1], bridgeMac[2],
                  bridgeMac[3], bridgeMac[4], bridgeMac[5]);
#else
    esp_read_mac(bridgeMac, ESP_MAC_WIFI_STA);
    bool macIsZero = true;
    for (int i = 0; i < 6; i++) if (bridgeMac[i]) { macIsZero = false; break; }
    if (macIsZero) {
        memcpy(bridgeMac, CUSTOM_MAC, 6);
        esp_base_mac_addr_set(bridgeMac);
    }
    Serial.printf("Bridge MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  bridgeMac[0], bridgeMac[1], bridgeMac[2],
                  bridgeMac[3], bridgeMac[4], bridgeMac[5]);
#endif
}

// =============================================================================
//  Tritium packet builders (UDP)
// =============================================================================

static void writeBusId(uint8_t* buf, uint16_t busNumber) {
    // 64-bit aligned: leading zero byte, then 5-byte V2 magic, then 2-byte bus#
    buf[0] = 0x00;
    memcpy(buf + 1, TRITIUM_V2_MAGIC, 5);
    buf[6] = (busNumber >> 8) & 0xFF;
    buf[7] =  busNumber       & 0xFF;
}

static void writeClientId(uint8_t* buf, const uint8_t* mac) {
    // 64-bit aligned: leading zero byte, then 1 more zero, then 6-byte MAC
    buf[0] = 0x00;
    buf[1] = 0x00;
    memcpy(buf + 2, mac, 6);
}

static int buildUdpPacket(uint8_t* buf,
                          uint32_t canId, uint8_t flags,
                          uint8_t length, const uint8_t* data) {
    writeBusId(buf, BRIDGE_BUS_NUMBER);          // bytes 0..7
    writeClientId(buf + 8, bridgeMac);           // bytes 8..15

    buf[16] = (canId >> 24) & 0xFF;
    buf[17] = (canId >> 16) & 0xFF;
    buf[18] = (canId >>  8) & 0xFF;
    buf[19] =  canId        & 0xFF;
    buf[20] = flags;
    buf[21] = length & 0x0F;

    memset(buf + 22, 0, 8);
    if (data && length > 0) {
        memcpy(buf + 22, data, length > 8 ? 8 : length);
    }
    return UDP_PKT_LEN;
}

static int buildHeartbeatPacket(uint8_t* buf) {
    uint8_t hbData[8];
    hbData[0] = (CAN_BITRATE_KBPS >> 8) & 0xFF;
    hbData[1] =  CAN_BITRATE_KBPS       & 0xFF;
    memcpy(hbData + 2, bridgeMac, 6);
    return buildUdpPacket(buf, 0x000, FLAG_HEARTBEAT, 8, hbData);
}

// Build just the 14-byte CAN frame portion (used for TCP sends)
static void buildTcpFrame(uint8_t* buf,
                          uint32_t canId, uint8_t flags,
                          uint8_t length, const uint8_t* data) {
    buf[0] = (canId >> 24) & 0xFF;
    buf[1] = (canId >> 16) & 0xFF;
    buf[2] = (canId >>  8) & 0xFF;
    buf[3] =  canId        & 0xFF;
    buf[4] = flags;
    buf[5] = length & 0x0F;
    memset(buf + 6, 0, 8);
    if (data && length > 0) {
        memcpy(buf + 6, data, length > 8 ? 8 : length);
    }
}

// =============================================================================
//  CAN bus
// =============================================================================

static bool initCan() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config  = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config  = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
    if (twai_start() != ESP_OK) return false;
    Serial.printf("CAN started: %d kbit/s\n", CAN_BITRATE_KBPS);
    return true;
}

static bool inFwdRange(uint32_t canId) {
    if (tcpFwdRange == 0) return false;            // no range configured
    return (canId >= tcpFwdId) && (canId < tcpFwdId + tcpFwdRange);
}

// Drain CAN frames and forward via TCP (preferred when client connected) or UDP
static void pumpCanOut() {
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        canRxCount++;

        uint8_t flags = 0;
        if (msg.flags & TWAI_MSG_FLAG_EXTD) flags |= FLAG_EXTENDED_ID;
        if (msg.flags & TWAI_MSG_FLAG_RTR)  flags |= FLAG_RTR;

        bool sentViaTcp = false;
        if (tcpClient && tcpClient.connected() && tcpHeaderDone &&
            inFwdRange(msg.identifier)) {
            uint8_t frame[TCP_FRAME_LEN];
            buildTcpFrame(frame, msg.identifier, flags,
                          msg.data_length_code, msg.data);
            if (tcpClient.write(frame, TCP_FRAME_LEN) == TCP_FRAME_LEN) {
                tcpTxCount++;
                sentViaTcp = true;
            }
        }

        if (!sentViaTcp) {
            uint8_t buf[UDP_PKT_LEN];
            int len = buildUdpPacket(buf, msg.identifier, flags,
                                     msg.data_length_code, msg.data);
            if (udp.writeTo(buf, len, TRITIUM_MCAST_IP, TRITIUM_PORT)) {
                udpTxCount++;
            }
        }
    }
}

// Inject a parsed CAN frame onto the physical CAN bus
static void forwardToCan(uint32_t id, uint8_t flags,
                         uint8_t length, const uint8_t* data) {
    twai_message_t msg = {};
    msg.identifier       = id;
    msg.data_length_code = (length > 8) ? 8 : length;
    if (flags & FLAG_EXTENDED_ID) msg.flags |= TWAI_MSG_FLAG_EXTD;
    if (flags & FLAG_RTR)         msg.flags |= TWAI_MSG_FLAG_RTR;
    memcpy(msg.data, data, msg.data_length_code);

    if (twai_transmit(&msg, pdMS_TO_TICKS(50)) == ESP_OK) {
        canTxCount++;
    }
}

// =============================================================================
//  UDP receive (kept as fallback)
// =============================================================================

static void onUdpPacket(AsyncUDPPacket& packet) {
    udpRxCount++;
    uint8_t* data = packet.data();
    size_t   len  = packet.length();

    if (len < UDP_PKT_LEN) return;
    // V2 magic starts at byte 1 (byte 0 is the 64-bit alignment padding)
    if (memcmp(data + 1, TRITIUM_V2_MAGIC, 5) != 0) return;
    uint16_t busNum = ((uint16_t)data[6] << 8) | data[7];
    if (busNum != BRIDGE_BUS_NUMBER) return;

    // Drop our own packets - our MAC is at offset 10 (8 bus + 2 client padding)
    if (memcmp(data + 10, bridgeMac, 6) == 0) return;

    size_t offset = 16;  // skip 8 bus_id + 8 client_id
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
//  TCP receive
// =============================================================================

static void resetTcpState() {
    tcpHeaderDone = false;
    tcpRxLen      = 0;
    tcpFwdId      = 0;
    tcpFwdRange   = 0;
}

static void parseTcpHeader() {
    // 4 bytes fwd_id (BE) + 4 bytes fwd_range (BE) + 8 bus_id + 8 client_id
    tcpFwdId    = ((uint32_t)tcpRxBuf[0] << 24) |
                  ((uint32_t)tcpRxBuf[1] << 16) |
                  ((uint32_t)tcpRxBuf[2] <<  8) |
                   (uint32_t)tcpRxBuf[3];
    tcpFwdRange = ((uint32_t)tcpRxBuf[4] << 24) |
                  ((uint32_t)tcpRxBuf[5] << 16) |
                  ((uint32_t)tcpRxBuf[6] <<  8) |
                   (uint32_t)tcpRxBuf[7];
    // bus_id at [8..15], client_id at [16..23] - we don't need to validate them
    Serial.printf("TCP handshake: fwd_id=0x%08X fwd_range=0x%08X\n",
                  tcpFwdId, tcpFwdRange);
    tcpHeaderDone = true;
}

static void parseTcpFrame() {
    uint32_t canId = ((uint32_t)tcpRxBuf[0] << 24) |
                     ((uint32_t)tcpRxBuf[1] << 16) |
                     ((uint32_t)tcpRxBuf[2] <<  8) |
                      (uint32_t)tcpRxBuf[3];
    uint8_t  flags  = tcpRxBuf[4];
    uint8_t  length = tcpRxBuf[5] & 0x0F;
    const uint8_t* frameData = tcpRxBuf + 6;
    if (!(flags & FLAG_HEARTBEAT) && !(flags & FLAG_SETTINGS)) {
        forwardToCan(canId, flags, length, frameData);
    }
}

static void serviceTcp() {
    // Accept new client if one is waiting
    if (tcpServer.hasClient()) {
        if (tcpClient && tcpClient.connected()) {
            // Already have a client; reject newcomer
            WiFiClient extra = tcpServer.available();
            extra.stop();
        } else {
            tcpClient = tcpServer.available();
            tcpClient.setNoDelay(true);
            resetTcpState();
            Serial.printf("TCP client connected: %s\n",
                          tcpClient.remoteIP().toString().c_str());
        }
    }

    if (!tcpClient || !tcpClient.connected()) {
        if (tcpClient) {
            Serial.println("TCP client disconnected");
            tcpClient.stop();
            resetTcpState();
        }
        return;
    }

    // Read available bytes into our buffer
    while (tcpClient.available() > 0 && tcpRxLen < sizeof(tcpRxBuf)) {
        int b = tcpClient.read();
        if (b < 0) break;
        tcpRxBuf[tcpRxLen++] = (uint8_t)b;
        tcpRxCount++;
    }

    // Drain buffer
    while (true) {
        if (!tcpHeaderDone) {
            if (tcpRxLen < TCP_HEADER_LEN) break;
            parseTcpHeader();
            // Shift out the 22 header bytes
            size_t remaining = tcpRxLen - TCP_HEADER_LEN;
            memmove(tcpRxBuf, tcpRxBuf + TCP_HEADER_LEN, remaining);
            tcpRxLen = remaining;
        } else {
            if (tcpRxLen < TCP_FRAME_LEN) break;
            parseTcpFrame();
            size_t remaining = tcpRxLen - TCP_FRAME_LEN;
            memmove(tcpRxBuf, tcpRxBuf + TCP_FRAME_LEN, remaining);
            tcpRxLen = remaining;
        }
    }
}

// =============================================================================
//  WiFi
// =============================================================================

static void connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    delay(100);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("Connecting to '%s'", WIFI_SSID);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (millis() - start > 30000) {
            Serial.println("\nWiFi timeout - rebooting");
            ESP.restart();
        }
    }
    Serial.printf("\nWiFi connected: IP=%s, RSSI=%d\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
}

// =============================================================================
//  Setup / loop
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(500);
    esp_log_level_set("*", ESP_LOG_ERROR);

    Serial.println("\n=== Prohelion CAN-Ethernet Bridge (TCP+UDP) ===");
    Serial.printf("Bus number: %u, Bitrate: %u kbit/s\n",
                  BRIDGE_BUS_NUMBER, CAN_BITRATE_KBPS);

    setupBridgeMac();
    connectWiFi();

    if (!initCan()) {
        Serial.println("CAN init failed - halting");
        while (1) delay(1000);
    }

    if (udp.listenMulticast(TRITIUM_MCAST_IP, TRITIUM_PORT)) {
        udp.onPacket(onUdpPacket);
        Serial.printf("UDP listening on %s:%u\n",
                      TRITIUM_MCAST_IP.toString().c_str(), TRITIUM_PORT);
    }

    tcpServer.begin();
    tcpServer.setNoDelay(true);
    Serial.printf("TCP server listening on port %u\n", TRITIUM_PORT);
    Serial.printf("\n>>> Configure Profinity to TCP-connect to %s:%u <<<\n\n",
                  WiFi.localIP().toString().c_str(), TRITIUM_PORT);
}

void loop() {
    pumpCanOut();
    serviceTcp();

    uint32_t now = millis();

    if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeatMs = now;
        uint8_t buf[UDP_PKT_LEN];
        int len = buildHeartbeatPacket(buf);
        if (udp.writeTo(buf, len, TRITIUM_MCAST_IP, TRITIUM_PORT)) {
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
                case TWAI_STATE_RUNNING:    state = "RUN";  break;
                case TWAI_STATE_STOPPED:    state = "STOP"; break;
                case TWAI_STATE_BUS_OFF:    state = "BOFF"; break;
                case TWAI_STATE_RECOVERING: state = "REC";  break;
            }
            busErrs = st.bus_error_count;
        }
        bool tcp = tcpClient && tcpClient.connected();
        Serial.printf("[%lus] CAN %s err=%u rx=%u tx=%u | UDP rx=%u tx=%u | TCP %s rx=%u tx=%u | RSSI=%d\n",
                      now / 1000, state, busErrs,
                      canRxCount, canTxCount,
                      udpRxCount, udpTxCount,
                      tcp ? "UP" : "DN",
                      tcpRxCount, tcpTxCount,
                      WiFi.RSSI());
    }

    twai_status_info_t st;
    if (twai_get_status_info(&st) == ESP_OK) {
        if (st.state == TWAI_STATE_BUS_OFF)      twai_initiate_recovery();
        else if (st.state == TWAI_STATE_STOPPED) twai_start();
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi dropped - reconnecting");
        connectWiFi();
    }

    delay(1);
}
