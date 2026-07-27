/*
 * =============================================================================
 *  Prohelion / Tritium CAN-Ethernet Bridge emulator for ESP32
 *  - eduroam edition (WPA2-Enterprise / 802.1X PEAP-MSCHAPv2) -
 *  - configured for University of Sheffield -
 * =============================================================================
 *
 *  Hardware and protocol notes are identical to the standard build. The only
 *  thing this file changes is how WiFi is brought up: it uses 802.1X
 *  enterprise auth so it can join eduroam.
 *
 *  Sheffield eduroam settings (per IT Services docs):
 *    SSID:               eduroam
 *    EAP method:         PEAP
 *    Phase 2 auth:       MSCHAPv2
 *    Identity:           your_username@sheffield.ac.uk
 *    Anonymous identity: leave blank
 *    CA certificate:     unspecified (no validation)
 *
 *  Compatible with arduino-esp32 v2.x AND v3.x. The right header is selected
 *  automatically via __has_include.
 *
 *  SECURITY NOTE: This firmware does NOT validate the eduroam server cert.
 *  That follows the Sheffield IT Services guidance for end-user devices, but
 *  it does mean a rogue access point spoofing eduroam could capture your
 *  university password. Don't run this firmware on a board you leave
 *  unattended in a public place. For permanent installation, paste the
 *  current eduroam CA cert into EAP_CA_CERT below and set EAP_USE_CA_CERT=1.
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
#include "esp_random.h"

// arduino-esp32 v3.x (ESP-IDF v5+) uses esp_eap_client.h
// arduino-esp32 v2.x (ESP-IDF v4.x) uses esp_wpa2.h
#if __has_include("esp_eap_client.h")
  #include "esp_eap_client.h"
  #define EAP_API_V3 1
#else
  #include "esp_wpa2.h"
  #define EAP_API_V3 0
#endif

// =============================================================================
//  USER CONFIGURATION  -  edit these for your setup
// =============================================================================

// Eduroam SSID (always literally "eduroam")
static const char* EAP_SSID     = "eduroam";

// Your University of Sheffield username followed by @sheffield.ac.uk
// e.g. "abc23xyz@sheffield.ac.uk"
static const char* EAP_USERNAME = "ACB24AAW@sheffield.ac.uk";

// Your University password
static const char* EAP_PASSWORD = "Hammer#07A";

// Outer identity. Sheffield docs say to leave anonymous identity blank,
// but most ESP32 stacks need *something* here. Setting it equal to the
// inner identity is the standard fallback and is what Sheffield's own
// CAT (eduroam Configuration Assistant Tool) profiles do internally.
static const char* EAP_IDENTITY = "ACB24AAW@sheffield.ac.uk";

// CA certificate validation. Sheffield IT Services say leave unspecified,
// so this is OFF by default. To enable: paste the AddTrust/USERTrust root
// or the current eduroam CA into EAP_CA_CERT and set this to 1.
#define EAP_USE_CA_CERT 0
static const char* EAP_CA_CERT = NULL;

// CAN bus configuration
#define CAN_BITRATE_KBPS    500
#define CAN_TX_GPIO         GPIO_NUM_5
#define CAN_RX_GPIO         GPIO_NUM_4

// Bridge identity. Bus number must be unique on your LAN.
#define BRIDGE_BUS_NUMBER   13

// =============================================================================
//  Tritium Bridge Protocol Constants
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
//  MAC handling - some clone ESP32 modules have no factory MAC in eFuse.
//  In that case we generate a locally-administered MAC and force the WiFi
//  stack to use it, otherwise EAPOL frames from 00:00:00:00:00:00 will be
//  rejected by the eduroam RADIUS server.
// =============================================================================

static void setupBridgeMac() {
    esp_read_mac(bridgeMac, ESP_MAC_WIFI_STA);

    bool macIsZero = true;
    for (int i = 0; i < 6; i++) {
        if (bridgeMac[i] != 0) { macIsZero = false; break; }
    }

    if (macIsZero) {
        Serial.println("WARN: eFuse MAC is empty. Generating fallback MAC.");
        // Locally-administered, unicast: first byte b'xxxxxx10'
        uint32_t r1 = esp_random();
        uint32_t r2 = esp_random();
        bridgeMac[0] = 0x02;
        bridgeMac[1] = (r1 >>  0) & 0xFF;
        bridgeMac[2] = (r1 >>  8) & 0xFF;
        bridgeMac[3] = (r1 >> 16) & 0xFF;
        bridgeMac[4] = (r2 >>  0) & 0xFF;
        bridgeMac[5] = (r2 >>  8) & 0xFF;

        // Set as the base MAC for the entire chip BEFORE WiFi.mode() runs.
        // This makes the WiFi stack use it for STA mode automatically.
        esp_err_t err = esp_base_mac_addr_set(bridgeMac);
        if (err != ESP_OK) {
            Serial.printf("WARN: esp_base_mac_addr_set failed (0x%x)\n", err);
        }
    }

    Serial.printf("Bridge MAC: %02X:%02X:%02X:%02X:%02X:%02X%s\n",
                  bridgeMac[0], bridgeMac[1], bridgeMac[2],
                  bridgeMac[3], bridgeMac[4], bridgeMac[5],
                  macIsZero ? "  (generated)" : "  (eFuse)");
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

    // Drop our own packets
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
//  WiFi - eduroam (WPA2-Enterprise / PEAP-MSCHAPv2)
// =============================================================================

static void connectEduroam() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    delay(100);

    Serial.printf("Connecting to eduroam as %s\n", EAP_USERNAME);

#if EAP_API_V3
    // arduino-esp32 v3.x / ESP-IDF v5+
    esp_eap_client_set_identity((uint8_t*)EAP_IDENTITY, strlen(EAP_IDENTITY));
    esp_eap_client_set_username((uint8_t*)EAP_USERNAME, strlen(EAP_USERNAME));
    esp_eap_client_set_password((uint8_t*)EAP_PASSWORD, strlen(EAP_PASSWORD));
  #if EAP_USE_CA_CERT
    if (EAP_CA_CERT) {
        esp_eap_client_set_ca_cert((uint8_t*)EAP_CA_CERT, strlen(EAP_CA_CERT));
    }
  #else
    esp_eap_client_set_disable_time_check(true);
  #endif
    esp_wifi_sta_enterprise_enable();
#else
    // arduino-esp32 v2.x / ESP-IDF v4.x
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t*)EAP_IDENTITY, strlen(EAP_IDENTITY));
    esp_wifi_sta_wpa2_ent_set_username((uint8_t*)EAP_USERNAME, strlen(EAP_USERNAME));
    esp_wifi_sta_wpa2_ent_set_password((uint8_t*)EAP_PASSWORD, strlen(EAP_PASSWORD));
  #if EAP_USE_CA_CERT
    if (EAP_CA_CERT) {
        esp_wifi_sta_wpa2_ent_set_ca_cert((uint8_t*)EAP_CA_CERT, strlen(EAP_CA_CERT));
    }
  #endif
    esp_wifi_sta_wpa2_ent_enable();
#endif

    WiFi.begin(EAP_SSID);

    Serial.print("WiFi connecting");
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (millis() - start > 60000) {
            Serial.println("\nWiFi timeout - rebooting");
            ESP.restart();
        }
    }
    Serial.printf("\nWiFi connected: SSID=%s, IP=%s, RSSI=%d\n",
                  EAP_SSID,
                  WiFi.localIP().toString().c_str(),
                  WiFi.RSSI());
}

// =============================================================================
//  Setup / loop
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(500);

    // Quiet the noisy ESP-IDF WPA/WiFi debug logs that were producing the
    // garbled characters in the serial monitor. Bump back up to ESP_LOG_INFO
    // if you need to debug an authentication failure.
    esp_log_level_set("*", ESP_LOG_ERROR);

    Serial.println("\n=== Prohelion CAN-Ethernet Bridge Emulator (eduroam) ===");
    Serial.printf("Bus number: %u, Bitrate: %u kbit/s\n",
                  BRIDGE_BUS_NUMBER, CAN_BITRATE_KBPS);
#if EAP_API_V3
    Serial.println("EAP API: v3 (esp_eap_client)");
#else
    Serial.println("EAP API: v2 (esp_wpa2)");
#endif

    // MAC must be sorted out BEFORE WiFi.mode(WIFI_STA), so the WiFi stack
    // picks up the fallback MAC if eFuse is empty.
    setupBridgeMac();

    connectEduroam();

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
        connectEduroam();
    }

    delay(1);
}
