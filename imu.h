/*
 * IMU Processing with Complementary Filter
 * Uses Arduino_LSM6DS3 for Nano IoT 33
 */

#ifndef IMU_H
#define IMU_H

#include <Arduino_LSM6DS3.h>
#include "config.h"
#include "types.h"

// Global sensor data (extern declaration)
extern SensorData sensors;

// Calibration offsets
static float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
static float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;

// Calibrate IMU - MUST keep drone still during this!
void calibrateIMU() {
    float gx = 0, gy = 0, gz = 0;
    float ax = 0, ay = 0, az = 0;
    int samples = 0;
    
    for (int i = 0; i < IMU_CALIBRATION_SAMPLES; i++) {
        if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
            float gxr, gyr, gzr, axr, ayr, azr;
            IMU.readGyroscope(gxr, gyr, gzr);
            IMU.readAcceleration(axr, ayr, azr);
            
            gx += gxr;
            gy += gyr;
            gz += gzr;
            ax += axr;
            ay += ayr;
            az += azr;
            samples++;
        }
        delay(2);
    }
    
    if (samples > 0) {
        gyroOffsetX = gx / samples;
        gyroOffsetY = gy / samples;
        gyroOffsetZ = gz / samples;
        
        // For accelerometer, we want 0,0,1 when level
        accelOffsetX = (ax / samples);
        accelOffsetY = (ay / samples);
        accelOffsetZ = (az / samples) - 1.0f;  // Subtract 1g
    }
    
    // Initialize angles to level
    sensors.roll = 0;
    sensors.pitch = 0;
    sensors.yaw = 0;
}

// Update IMU and calculate attitude
void updateIMU(float dt) {
    if (!IMU.gyroscopeAvailable() || !IMU.accelerationAvailable()) {
        return;
    }
    
    float gx, gy, gz;
    float ax, ay, az;
    
    // Read raw values
    IMU.readGyroscope(gx, gy, gz);
    IMU.readAcceleration(ax, ay, az);
    
    // Apply calibration offsets
    gx -= gyroOffsetX;
    gy -= gyroOffsetY;
    gz -= gyroOffsetZ;
    ax -= accelOffsetX;
    ay -= accelOffsetY;
    az -= accelOffsetZ;
    
    // Store raw values
    sensors.gyroX = gx;
    sensors.gyroY = gy;
    sensors.gyroZ = gz;
    sensors.accelX = ax;
    sensors.accelY = ay;
    sensors.accelZ = az;
    
    // Calculate accelerometer angles
    float accelRoll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / PI;
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
    
    // Complementary filter
    // Trust gyro for short term (fast changes), accel for long term (drift correction)
    sensors.roll = COMPLEMENTARY_FILTER_ALPHA * (sensors.roll + gx * dt) + 
                   (1.0f - COMPLEMENTARY_FILTER_ALPHA) * accelRoll;
    
    sensors.pitch = COMPLEMENTARY_FILTER_ALPHA * (sensors.pitch + gy * dt) + 
                    (1.0f - COMPLEMENTARY_FILTER_ALPHA) * accelPitch;
    
    // Yaw uses gyro integration only (no magnetometer)
    // Could add magnetometer later for absolute heading
    sensors.yaw += gz * dt;
    
    // Keep yaw in -180 to 180 range
    if (sensors.yaw > 180.0f) sensors.yaw -= 360.0f;
    if (sensors.yaw < -180.0f) sensors.yaw += 360.0f;
    
    // Store yaw rate for yaw PID
    sensors.yawRate = gz;
}

#endif // IMU_H
