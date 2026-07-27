#pragma once

#include <Arduino.h>
#include "config.h"

typedef enum : uint8_t {
    STATUS_OK            = 0,  // Good reading, all or most sensors healthy
    STATUS_OK_DEGRADED   = 1,  // Good reading, but some sensors excluded
    ERROR_NO_MAGNET      = 2,  // Signal present but below detection threshold
    ERROR_COVERAGE       = 3,  // Not enough valid sensors to compute position
    ERROR_SIGNAL_ANOMALY = 4,  // Bimodal or implausible signal (wrong magnet?)
    ERROR_OUT_OF_RANGE   = 5,  // Computed position outside physical array bounds
    ERROR_ALL_DEAD       = 6,  // Every sensor is reporting a fault
} PositionStatus;

// Individual sensor fault types
typedef enum : uint8_t {
    SENSOR_OK           = 0,
    FAULT_RAIL_HIGH     = 1,  // Stuck at max — possible short to VCC / broken sensor
    FAULT_RAIL_LOW      = 2,  // Stuck at zero — disconnected / broken sensor
    FAULT_NOISE         = 3,  // Erratic readings — interference / bad connection
} SensorFault;


typedef struct {
    float           position_mm;    // Computed position, or NAN on error
    PositionStatus  status;         // STATUS_OK or an ERROR_* code
    uint16_t        sensor_health;  // Bitmask: bit N=1 means sensor N is valid
    uint8_t         valid_count;    // How many sensors contributed to this reading
} PositionResult;

class HallPositionSensor {
public:
    HallPositionSensor();

    // Call once in setup(). Configures GPIO and ADC.
    void begin();

    // Measure ambient baseline with no magnet present.
    // Call after begin(), before the main loop.
    void calibrate();

    // Read all sensors and compute position. Call every loop iteration.
    // Returns a PositionResult — always check .status before using .position_mm.
    PositionResult update();

    // Returns the stored baseline for sensor i (useful for diagnostics).
    uint16_t getBaseline(uint8_t i) const { return _baseline[i]; }

    // Returns the raw (oversampled) last reading for sensor i.
    uint16_t getRaw(uint8_t i) const { return _raw[i]; }

    // Returns the signal above baseline for sensor i (clamped to 0).
    uint16_t getSignal(uint8_t i) const { return _signal[i]; }

    // Returns the fault status for sensor i.
    SensorFault getFault(uint8_t i) const { return _fault[i]; }

private:
    uint16_t   _baseline[N_SENSORS];
    uint16_t   _raw[N_SENSORS];
    uint16_t   _signal[N_SENSORS];
    SensorFault _fault[N_SENSORS];
    uint8_t    _stuckCount[N_SENSORS];  // consecutive stuck-rail read counter

    void     _setMuxChannel(uint8_t ch);
    uint16_t _readADC();
    uint16_t _oversampledRead(uint8_t muxCh);
    void     _validateSensors();
    bool     _checkCoverage(uint8_t &validCount, uint16_t &healthMask);
    float    _weightedCentroid(uint16_t healthMask);
    bool     _isSignalAnomalous(uint16_t healthMask);
};

HallPositionSensor::HallPositionSensor() {
    memset(_baseline,   0, sizeof(_baseline));
    memset(_raw,        0, sizeof(_raw));
    memset(_signal,     0, sizeof(_signal));
    memset(_fault,      0, sizeof(_fault));
    memset(_stuckCount, 0, sizeof(_stuckCount));
}

void HallPositionSensor::begin() {
    // Configure mux select pins as outputs
    pinMode(MUX_S0_PIN, OUTPUT);
    pinMode(MUX_S1_PIN, OUTPUT);
    pinMode(MUX_S2_PIN, OUTPUT);
    pinMode(MUX_S3_PIN, OUTPUT);

    // Configure ADC
    // 12-bit resolution (0–4095), 11dB attenuation for 0–3.3V range
    analogReadResolution(12);
    analogSetPinAttenuation(MUX_SIG_PIN, ADC_11db);

    // Set a default flat baseline so the sensor is usable before calibrate()
    for (int i = 0; i < N_SENSORS; i++) {
        _baseline[i] = 2048;
    }
}

void HallPositionSensor::_setMuxChannel(uint8_t ch) {
    digitalWrite(MUX_S0_PIN, (ch >> 0) & 1);
    digitalWrite(MUX_S1_PIN, (ch >> 1) & 1);
    digitalWrite(MUX_S2_PIN, (ch >> 2) & 1);
    digitalWrite(MUX_S3_PIN, (ch >> 3) & 1);
}

uint16_t HallPositionSensor::_readADC() {
    return (uint16_t)analogRead(MUX_SIG_PIN);
}

uint16_t HallPositionSensor::_oversampledRead(uint8_t muxCh) {
    _setMuxChannel(muxCh);
    delayMicroseconds(SETTLE_US);

    uint32_t acc = 0;
    for (int s = 0; s < OVERSAMPLE; s++) {
        acc += _readADC();
    }
    uint16_t avg = (uint16_t)(acc / OVERSAMPLE);
    Serial.printf("  ch %d: %d\n", muxCh, avg);
    return avg;
}

void HallPositionSensor::calibrate() {
    for (int i = 0; i < N_SENSORS; i++) {
        _setMuxChannel(SENSOR_MUX_CH[i]);
        delayMicroseconds(SETTLE_US * 2);
        
        uint32_t acc = 0;
        for (int s = 0; s < BASELINE_SAMPLES; s++) {
            acc += _readADC();
            delayMicroseconds(10);
        }
        _baseline[i] = (uint16_t)(acc / BASELINE_SAMPLES);
    }

    // Reset fault tracking after fresh calibration
    memset(_fault,      0, sizeof(_fault));
    memset(_stuckCount, 0, sizeof(_stuckCount));
}

void HallPositionSensor::_validateSensors() {
    for (int i = 0; i < N_SENSORS; i++) {
        uint16_t v = _raw[i];

        if (v >= RAIL_HIGH_THRESH) {
            _stuckCount[i]++;
            if (_stuckCount[i] >= STUCK_COUNT) {
                _fault[i] = FAULT_RAIL_HIGH;
            }
        } else if (v <= RAIL_LOW_THRESH) {
            _stuckCount[i]++;
            if (_stuckCount[i] >= STUCK_COUNT) {
                _fault[i] = FAULT_RAIL_LOW;
            }
        } else {
            // Reading looks normal — reset stuck counter and clear fault
            _stuckCount[i] = 0;
            _fault[i] = SENSOR_OK;
        }

        // Compute signal above baseline, clamped to zero
        if (_fault[i] == SENSOR_OK) {
            int32_t sig = (int32_t)v - (int32_t)_baseline[i];
            _signal[i] = (uint16_t)abs(sig);
        } else {
            _signal[i] = 0;
        }
    }
}

bool HallPositionSensor::_checkCoverage(uint8_t &validCount, uint16_t &healthMask) {
    validCount = 0;
    healthMask = 0;

    for (int i = 0; i < N_SENSORS; i++) {
        if (_fault[i] == SENSOR_OK) {
            validCount++;
            healthMask |= (1 << i);
        }
    }

    if (validCount < MIN_VALID_SENSORS) return false;

    // Check there is at least one contiguous run of MIN_VALID_SENSORS
    int run = 0;
    for (int i = 0; i < N_SENSORS; i++) {
        if ((healthMask >> i) & 1) {
            run++;
            if (run >= MIN_VALID_SENSORS) return true;
        } else {
            run = 0;
        }
    }
    return false;
}

float HallPositionSensor::_weightedCentroid(uint16_t healthMask) {
    double weightedSum = 0.0;
    double totalWeight = 0.0;

    for (int i = 0; i < N_SENSORS; i++) {
        if (!((healthMask >> i) & 1)) continue;
        double w = (double)_signal[i] * (double)_signal[i];  // squared weight
        weightedSum += SENSOR_POS_MM[i] * w;
        totalWeight += w;
    }

    if (totalWeight < 1.0) return NAN;
    return (float)(weightedSum / totalWeight);
}

bool HallPositionSensor::_isSignalAnomalous(uint16_t healthMask) {
    // Find the peak sensor
    int peakIdx = -1;
    uint16_t peakVal = 0;
    for (int i = 0; i < N_SENSORS; i++) {
        if (((healthMask >> i) & 1) && _signal[i] > peakVal) {
            peakVal = _signal[i];
            peakIdx = i;
        }
    }
    if (peakIdx < 0) return true;

    // Check for a second peak far from the first (bimodal = two magnets or
    // reflections). A second peak counts if it's > 40% of the main peak
    // AND more than 2 sensors away.
    for (int i = 0; i < N_SENSORS; i++) {
        if (!((healthMask >> i) & 1)) continue;
        if (abs(i - peakIdx) <= 2) continue;  // adjacent sensors — normal rolloff
        if (_signal[i] > peakVal * 0.40f) return true;
    }
    return false;
}

PositionResult HallPositionSensor::update() {
    PositionResult result;
    result.position_mm  = NAN;
    result.status       = STATUS_OK;
    result.sensor_health = 0;
    result.valid_count  = 0;

    // --- Step 1: Read all sensors ---
    for (int i = 0; i < N_SENSORS; i++) {
        _raw[i] = _oversampledRead(SENSOR_MUX_CH[i]);
    }

    // --- Step 2: Per-sensor validation ---
    _validateSensors();

    // --- Step 3: Coverage check ---
    uint8_t  validCount;
    uint16_t healthMask;

    if (!_checkCoverage(validCount, healthMask)) {
        result.valid_count  = validCount;
        result.sensor_health = healthMask;
        result.status = (validCount == 0) ? ERROR_ALL_DEAD : ERROR_COVERAGE;
        return result;
    }

    result.valid_count   = validCount;
    result.sensor_health = healthMask;

    // --- Step 4: Check if a magnet is actually present ---
    uint16_t peakSignal = 0;
    for (int i = 0; i < N_SENSORS; i++) {
        if (_signal[i] > peakSignal) peakSignal = _signal[i];
    }

    if (peakSignal < MAGNET_DETECT_THRESHOLD) {
        result.status = ERROR_NO_MAGNET;
        return result;
    }

    // --- Step 5: Anomaly check ---
    if (_isSignalAnomalous(healthMask)) {
        result.status = ERROR_SIGNAL_ANOMALY;
        return result;
    }

    // --- Step 6: Weighted centroid ---
    float pos = _weightedCentroid(healthMask);

    if (isnan(pos)) {
        result.status = ERROR_SIGNAL_ANOMALY;
        return result;
    }

    // --- Step 7: Bounds check ---
    float arrayMin = SENSOR_POS_MM[0];
    float arrayMax = SENSOR_POS_MM[N_SENSORS - 1];
    if (pos < arrayMin - 5.0f || pos > arrayMax + 5.0f) {
        result.status = ERROR_OUT_OF_RANGE;
        return result;
    }

    // --- Step 8: Set final status ---
    result.position_mm = pos;
    result.status = (validCount < N_SENSORS) ? STATUS_OK_DEGRADED : STATUS_OK;
    return result;
}
