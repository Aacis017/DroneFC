/*
 * DroneFC Configuration
 * 
 * Edit these values to tune your drone
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================
// MOTOR PIN CONFIGURATION
// ============================================
// Quadcopter X configuration
//     FL(2)   FR(5)
//        \   /
//         [X]
//        /   \
//     RL(3)   RR(9)

#define MOTOR_FL_PIN    2   // Front Left
#define MOTOR_FR_PIN    5   // Front Right
#define MOTOR_RL_PIN    3   // Rear Left
#define MOTOR_RR_PIN    9   // Rear Right

// ============================================
// ESC CONFIGURATION
// ============================================
#define ESC_MIN_PULSE       1000    // Minimum PWM (microseconds)
#define ESC_MAX_PULSE       2000    // Maximum PWM (microseconds)
#define ESC_ARM_PULSE       1050    // Arming pulse
#define ESC_IDLE_PULSE      1100    // Idle when armed

// ============================================
// SERIAL CONFIGURATION
// ============================================
#define FLIGHT_SERIAL       Serial1 // UART to Raspberry Pi
#define DEBUG_SERIAL        Serial  // USB for debugging
#define BAUD_RATE           115200

// ============================================
// CONTROL LOOP TIMING
// ============================================
#define LOOP_RATE_HZ        250     // Main loop frequency
#define LOOP_INTERVAL_US    (1000000 / LOOP_RATE_HZ)

#define BARO_RATE_HZ        40      // Barometer update rate
#define BARO_INTERVAL_US    (1000000 / BARO_RATE_HZ)

#define TELEMETRY_RATE_HZ   10      // Telemetry send rate
#define TELEMETRY_INTERVAL_US (1000000 / TELEMETRY_RATE_HZ)

// ============================================
// PID GAINS - ROLL (user's proven values)
// ============================================
#define PID_ROLL_P      3.55f
#define PID_ROLL_I      0.005f
#define PID_ROLL_D      2.05f
#define PID_ROLL_MAX    300.0f

// ============================================
// PID GAINS - PITCH (user's proven values)
// ============================================
#define PID_PITCH_P     3.55f
#define PID_PITCH_I     0.005f
#define PID_PITCH_D     2.05f
#define PID_PITCH_MAX   300.0f

// ============================================
// PID GAINS - YAW (user's proven values)
// ============================================
#define PID_YAW_P       1.8f
#define PID_YAW_I       0.002f
#define PID_YAW_D       1.0f
#define PID_YAW_MAX     300.0f

// ============================================
// PID GAINS - ALTITUDE (user's proven values)
// ============================================
#define PID_ALT_P       50.0f
#define PID_ALT_I       0.5f
#define PID_ALT_D       20.0f
#define PID_ALT_MAX     200.0f

// ============================================
// SAFETY LIMITS
// ============================================
#define MAX_ROLL_ANGLE      30.0f   // Maximum roll angle (degrees)
#define MAX_PITCH_ANGLE     30.0f   // Maximum pitch angle (degrees)
#define MAX_YAW_RATE        180.0f  // Maximum yaw rate (deg/s)

#define MIN_THROTTLE        1000    // Minimum throttle value
#define MAX_THROTTLE        2000    // Maximum throttle value
#define MIN_THROTTLE_FOR_PID 1150   // Minimum throttle for PID to engage

// ============================================
// IMU CALIBRATION
// ============================================
#define IMU_CALIBRATION_SAMPLES 500
#define COMPLEMENTARY_FILTER_ALPHA 0.98f  // Gyro weight (0.98 = trust gyro 98%)

// ============================================
// BAROMETER
// ============================================
#define BARO_CALIBRATION_SAMPLES 50
#define SEA_LEVEL_PRESSURE 101325.0f  // Pa (adjust for your location)

// ============================================
// ARMING
// ============================================
#define ARM_SAFETY_CHECK    true    // Check level before arming
#define MAX_ARM_ANGLE       10.0f   // Max angle to allow arming

#endif // CONFIG_H
