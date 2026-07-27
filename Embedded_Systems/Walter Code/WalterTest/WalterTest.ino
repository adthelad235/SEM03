#include <WalterModem.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include "WalterFeels.h"


// --- Hardware Wiring ---
#define EX_GNSS_RX_PIN 38 
#define EX_GNSS_TX_PIN 39 
#define GNSS_BAUD_RATE 9600 

// ---- GPS speed with smoothing + stationary deadband ----
#define GPS_SPEED_SMOOTH_N   1      // rolling average window
#define GPS_SPEED_DEADBAND   0.0f   // km/h below this reads as 0 (kills jitter)


WalterModem modem;
HardwareSerial GNSSSerial(1);
TinyGPSPlus gps;

unsigned long lastPrintTime = 0;

static float   s_speedBuf[GPS_SPEED_SMOOTH_N] = {0};
static uint8_t s_speedIdx = 0;
static uint8_t s_speedCount = 0;

// Call this regularly (e.g. once per loop). Returns smoothed km/h, or -1 if no fix.
float getGpsSpeedKmh() {
    if (!gps.speed.isValid() || !gps.location.isValid()) {
        // no valid fix - reset the smoother so stale values don't linger
        s_speedCount = 0;
        s_speedIdx   = 0;
        return -1;
    }

    float raw = gps.speed.kmph();

    // deadband: GPS reports small phantom speeds when stationary
    if (raw < GPS_SPEED_DEADBAND) raw = 0.0f;

    // push into the rolling buffer
    s_speedBuf[s_speedIdx] = raw;
    s_speedIdx = (s_speedIdx + 1) % GPS_SPEED_SMOOTH_N;
    if (s_speedCount < GPS_SPEED_SMOOTH_N) s_speedCount++;

    // average whatever we have so far
    float sum = 0;
    for (uint8_t i = 0; i < s_speedCount; i++) sum += s_speedBuf[i];
    return sum / s_speedCount;
}

void setup() {
  Serial.begin(115200);
  delay(3000); // Give you time to open the Serial Monitor
  Serial.println("\n--- Walter Hardware Local Test ---");
  WalterFeels::set3v3(true);
  Serial.println("3.3v connection open");

  // 1. Test Modem Connection (Proves Walter <-> Sequans chip works)
  Serial.print("Initializing LTE Modem... ");
  if (modem.begin(&Serial2)) {          // <-- v1.5.0: pass the modem's UART
      Serial.println("SUCCESS! Modem is responding.");

      // Define the PDP context (APN). Auth is now a SEPARATE call.
      modem.definePDPContext(1, "soracom.io");

      // Soracom auth: user/pass "sora"/"sora".
      // VERIFY in WalterModem.h -> search "setAuthParams"
      //modem.setAuthParams(1, "sora", "sora");

      Serial.println("Soracom APN configured. Modem searching for network in background.");
  } else {
      Serial.println("FAILED! Check power, SIM, and ensure the LTE antenna is connected.");
  }

  // 2. Test External GNSS Connection
  Serial.print("Initializing External GNSS on UART... ");
  GNSSSerial.begin(GNSS_BAUD_RATE, SERIAL_8N1, EX_GNSS_RX_PIN, EX_GNSS_TX_PIN);
  Serial.println("STARTED!");
  Serial.println("-----------------------------------");
}

void loop() {
  // Constantly read the data stream from the SparkFun module
  while (GNSSSerial.available()) {
    gps.encode(GNSSSerial.read()); 
  }

  // Print an update to the Serial Monitor every 3 seconds
  if (millis() - lastPrintTime > 250) {
    lastPrintTime = millis();

    Serial.print("[GNSS STATUS] ");
    
    // Check if we are receiving any data at all
    if (gps.charsProcessed() < 10) {
       Serial.println("ERROR: No data from SparkFun module. Check your 3.3V, GND, and RX/TX wiring!");
    } 
    // Check if we have data, but no satellite fix yet
    else if (!gps.location.isValid()) {
       Serial.print("Looking for satellites... (Found: ");
       Serial.print(gps.satellites.value());
       Serial.println(") -> Put the antenna near a window!");
    } 
    // Success: We have a live location
    else {
       Serial.print("FIX ACQUIRED! Lat: ");
       Serial.print(gps.location.lat(), 6);
       Serial.print(" | Lon: ");
       Serial.print(gps.location.lng(), 6);
       Serial.print(" | Sats: ");
       Serial.print(gps.satellites.value());
       Serial.print(" | Speed: ");
       Serial.print(getGpsSpeedKmh());
       Serial.println(" kmph");
    }
  }
}