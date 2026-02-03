/*
 * Sensor Management - BMP180 Barometer
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_BMP085.h>
#include "config.h"
#include "types.h"

// Global sensor data (extern declaration)
extern SensorData sensors;

// Barometer instance
static Adafruit_BMP085 bmp;
static bool baroAvailable = false;

// Initialize barometer
bool initBarometer() {
    baroAvailable = bmp.begin();
    return baroAvailable;
}

// Calibrate ground altitude
void calibrateAltitude() {
    if (!baroAvailable) return;
    
    float sum = 0;
    for (int i = 0; i < BARO_CALIBRATION_SAMPLES; i++) {
        sum += bmp.readAltitude(SEA_LEVEL_PRESSURE);
        delay(10);
    }
    sensors.groundAltitude = sum / BARO_CALIBRATION_SAMPLES;
}

// Update barometer reading
void updateBarometer() {
    if (!baroAvailable) return;
    
    static float lastAltitude = 0;
    static uint32_t lastTime = 0;
    
    sensors.pressure = bmp.readPressure();
    sensors.temperature = bmp.readTemperature();
    
    // Calculate altitude relative to ground
    float rawAltitude = bmp.readAltitude(SEA_LEVEL_PRESSURE);
    
    // Low pass filter for altitude
    float newAltitude = rawAltitude - sensors.groundAltitude;
    sensors.altitude = sensors.altitude * 0.9f + newAltitude * 0.1f;
    
    // Calculate vertical speed
    uint32_t now = millis();
    if (lastTime > 0) {
        float dt = (now - lastTime) / 1000.0f;
        if (dt > 0) {
            sensors.verticalSpeed = (sensors.altitude - lastAltitude) / dt;
        }
    }
    lastAltitude = sensors.altitude;
    lastTime = now;
}

#endif // SENSORS_H
