/*
 * DroneFC Type Definitions
 */

#ifndef TYPES_H
#define TYPES_H

// Flight modes
enum FlightMode {
    MODE_STABILIZE = 0,     // Manual with stabilization
    MODE_ALT_HOLD = 1,      // Altitude hold
    MODE_LAND = 2,          // Auto land
    MODE_TAKEOFF = 3        // Auto takeoff
};

// Flight state structure
struct FlightState {
    bool armed;
    FlightMode mode;
    
    // Stick inputs (from Pi)
    int16_t throttle;       // 1000-2000
    float targetRoll;       // degrees
    float targetPitch;      // degrees
    float targetYaw;        // degrees/sec for rate, degrees for heading
    float targetAltitude;   // meters
    
    // Auto operations
    bool takeoffInProgress;
    bool landingInProgress;
    float takeoffTargetAlt;
};

// Sensor data structure
struct SensorData {
    // IMU raw data
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    
    // Processed attitude (degrees)
    float roll;
    float pitch;
    float yaw;
    float yawRate;
    
    // Barometer
    float pressure;
    float temperature;
    float altitude;         // meters above ground
    float verticalSpeed;    // m/s
    
    // Ground reference
    float groundAltitude;   // meters (sea level at startup)
};

// Motor values
struct MotorValues {
    uint16_t fl;    // Front Left
    uint16_t fr;    // Front Right
    uint16_t rl;    // Rear Left
    uint16_t rr;    // Rear Right
};

#endif // TYPES_H
