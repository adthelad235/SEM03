/*
 * WiFi Diagnostic Sketch for ESP32
 *
 * Scans for nearby networks (so you can see whether your hotspot is
 * actually visible to the ESP32), then tries to connect and reports
 * exactly what fails.
 *
 * Edit WIFI_SSID and WIFI_PASSWORD, flash, watch serial monitor.
 */

#include <WiFi.h>
#include "esp_wifi.h"

static const char* WIFI_SSID     = "Tom's iPhone";
static const char* WIFI_PASSWORD = "tomwright";

const char* authToString(wifi_auth_mode_t a) {
    switch (a) {
        case WIFI_AUTH_OPEN:            return "OPEN";
        case WIFI_AUTH_WEP:             return "WEP";
        case WIFI_AUTH_WPA_PSK:         return "WPA-PSK";
        case WIFI_AUTH_WPA2_PSK:        return "WPA2-PSK";
        case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/WPA2-PSK";
        case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-Enterprise";
        case WIFI_AUTH_WPA3_PSK:        return "WPA3-PSK";
        case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/WPA3-PSK";
        default:                        return "?";
    }
}

const char* statusToString(wl_status_t s) {
    switch (s) {
        case WL_NO_SHIELD:       return "NO_SHIELD";
        case WL_IDLE_STATUS:     return "IDLE";
        case WL_NO_SSID_AVAIL:   return "NO_SSID_AVAIL (network not found)";
        case WL_SCAN_COMPLETED:  return "SCAN_COMPLETED";
        case WL_CONNECTED:       return "CONNECTED";
        case WL_CONNECT_FAILED:  return "CONNECT_FAILED (wrong password?)";
        case WL_CONNECTION_LOST: return "CONNECTION_LOST";
        case WL_DISCONNECTED:    return "DISCONNECTED";
        default:                 return "?";
    }
}

void scanNetworks() {
    Serial.println("\n--- Scanning for nearby networks (5 sec) ---");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    int n = WiFi.scanNetworks();
    if (n == 0) {
        Serial.println("No networks found at all. Check antenna/distance.");
        return;
    }

    Serial.printf("Found %d networks:\n", n);
    Serial.println("  RSSI  CH  Auth              SSID");
    Serial.println("  ----  --  ----------------  -------------------------");
    bool foundTarget = false;
    for (int i = 0; i < n; i++) {
        bool isTarget = (WiFi.SSID(i) == String(WIFI_SSID));
        if (isTarget) foundTarget = true;
        Serial.printf("%s %4d  %2d  %-16s  %s\n",
                      isTarget ? ">>" : "  ",
                      WiFi.RSSI(i),
                      WiFi.channel(i),
                      authToString(WiFi.encryptionType(i)),
                      WiFi.SSID(i).c_str());
    }
    Serial.println();
    if (foundTarget) {
        Serial.printf("OK: '%s' is visible.\n", WIFI_SSID);
    } else {
        Serial.printf("PROBLEM: '%s' is NOT visible to the ESP32.\n", WIFI_SSID);
        Serial.println("  - Is the hotspot on 2.4 GHz? (ESP32 cannot see 5 GHz)");
        Serial.println("  - Is the hotspot screen open on your phone?");
        Serial.println("  - Is the phone close enough to the ESP32?");
        Serial.println("  - Does the SSID exactly match (case-sensitive)?");
    }
    WiFi.scanDelete();
}

void tryConnect() {
    Serial.printf("\n--- Trying to connect to '%s' ---\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint32_t start = millis();
    wl_status_t lastStatus = WL_IDLE_STATUS;
    while (millis() - start < 30000) {
        wl_status_t s = WiFi.status();
        if (s != lastStatus) {
            Serial.printf("[%5lums] status: %s\n", millis() - start, statusToString(s));
            lastStatus = s;
        }
        if (s == WL_CONNECTED) {
            Serial.printf("\nSUCCESS: connected.\n");
            Serial.printf("  IP:   %s\n", WiFi.localIP().toString().c_str());
            Serial.printf("  RSSI: %d dBm\n", WiFi.RSSI());
            Serial.printf("  MAC:  %s\n", WiFi.macAddress().c_str());
            return;
        }
        if (s == WL_CONNECT_FAILED || s == WL_NO_SSID_AVAIL) {
            Serial.printf("\nFAILED: %s\n", statusToString(s));
            return;
        }
        delay(200);
    }
    Serial.printf("\nTIMEOUT after 30s. Final status: %s\n",
                  statusToString(WiFi.status()));
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== ESP32 WiFi Diagnostic ===");
    Serial.printf("Looking for: '%s'\n", WIFI_SSID);

    scanNetworks();
    tryConnect();

    Serial.println("\n--- Diagnostic complete ---");
}

void loop() {
    delay(1000);
}
