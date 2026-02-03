/*
 * DroneFC - Arduino Flight Controller
 * 
 * Based on MultiWii reference implementation
 * PID implementation follows industry-standard MultiWii algorithm
 * 
 * Hardware:
 * - Arduino Nano IoT 33 (built-in LSM6DS3 IMU)
 * - BMP180 barometer (I2C)
 * - 4x Brushless motors with ESCs
 * - Raspberry Pi Zero 2W (UART control)
 */

#include <Wire.h>
#include <Servo.h>
#include <ArduinoJson.h>
#include <Arduino_LSM6DS3.h>
#include <Adafruit_BMP085.h>

#include "config.h"
#include "types.h"

// ====== Motor objects ======
Servo motorFL, motorFR, motorRL, motorRR;

// ====== IMU Data ======
// Raw sensor data
int16_t gyroADC[3];    // Raw gyro readings [ROLL, PITCH, YAW]
int16_t accADC[3];     // Raw accelerometer readings
int16_t gyroZero[3] = {0, 0, 0};  // Gyro calibration offset

// Attitude estimation
int16_t angle[2] = {0, 0};  // Roll and Pitch angles in 0.1 degree units
int16_t heading = 0;         // Heading in degrees

// ====== PID Variables (MultiWii style) ======
// Error accumulators for integral term
int32_t errorGyroI[3] = {0, 0, 0};
int16_t errorAngleI[2] = {0, 0};

// Delta history for D-term smoothing (3-sample moving average)
int16_t delta1[3] = {0, 0, 0};
int16_t delta2[3] = {0, 0, 0};

// Last gyro readings for delta calculation
int16_t lastGyro[3] = {0, 0, 0};

// PID output per axis
int16_t axisPID[3];

// ====== PID Gains (stored as 8-bit scaled values like MultiWii) ======
// P8, I8, D8 format - scaled by different factors
typedef struct {
    uint8_t P8;
    uint8_t I8;
    uint8_t D8;
} pidGain_t;

pidGain_t pid[4];  // ROLL, PITCH, YAW, ALT

// ====== RC Command ======
int16_t rcCommand[4] = {0, 0, 0, 1000};  // ROLL, PITCH, YAW, THROTTLE

// ====== Control State ======
bool armed = false;
FlightMode currentMode = MODE_STABILIZE;
bool angleMode = true;   // Self-level mode

// ====== Timing ======
uint32_t currentTime = 0;
uint16_t previousTime = 0;
uint16_t cycleTime = 0;

// ====== Barometer ======
Adafruit_BMP085 bmp;
int32_t AltHold = 0;       // Target altitude in cm
int16_t BaroPID = 0;       // Altitude PID output
int16_t errorAltitudeI = 0;
float baselinePressure = 0;
int32_t EstAlt = 0;        // Estimated altitude in cm
int16_t vario = 0;         // Vertical speed in cm/s
bool barometerHealthy = false;

// ====== Safety ======
unsigned long lastCommandTime = 0;
bool imuHealthy = true;
bool calibratingG = false;
uint16_t calibratingGCount = 0;

// ====== Telemetry ======
unsigned long lastTelemetryTime = 0;

// ====== Constants ======
#define ROLL    0
#define PITCH   1
#define YAW     2
#define THROTTLE 3

#define GYRO_SCALE 16.4f   // LSM6DS3 at 2000dps sensitivity

// =========================================================
// ================= HELPER FUNCTIONS ======================
// =========================================================

// Fast integer multiply used in MultiWii
#define mul(a, b) ((int32_t)(a) * (int32_t)(b))

void initPIDGains() {
    // ROLL
    pid[ROLL].P8 = 40;      // P * 10
    pid[ROLL].I8 = 30;      // I * 1000
    pid[ROLL].D8 = 23;      // D * 1
    
    // PITCH (same as roll for symmetry)
    pid[PITCH].P8 = 40;
    pid[PITCH].I8 = 30;
    pid[PITCH].D8 = 23;
    
    // YAW
    pid[YAW].P8 = 85;
    pid[YAW].I8 = 45;
    pid[YAW].D8 = 0;
    
    // ALTITUDE (LEVEL mode)
    pid[3].P8 = 50;
    pid[3].I8 = 20;
    pid[3].D8 = 15;
}

void calibrateGyro() {
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    float gx, gy, gz;
    
    DEBUG_SERIAL.println(F("Calibrating gyro - keep STILL..."));
    calibratingG = true;
    
    for (int i = 0; i < IMU_CALIBRATION_SAMPLES; i++) {
        if (IMU.gyroscopeAvailable()) {
            IMU.readGyroscope(gx, gy, gz);
            // Convert to raw ADC-like values
            sumX += (int16_t)(gx * GYRO_SCALE);
            sumY += (int16_t)(gy * GYRO_SCALE);
            sumZ += (int16_t)(gz * GYRO_SCALE);
        }
        delay(2);
    }
    
    gyroZero[ROLL] = sumX / IMU_CALIBRATION_SAMPLES;
    gyroZero[PITCH] = sumY / IMU_CALIBRATION_SAMPLES;
    gyroZero[YAW] = sumZ / IMU_CALIBRATION_SAMPLES;
    
    calibratingG = false;
    angle[ROLL] = 0;
    angle[PITCH] = 0;
    
    DEBUG_SERIAL.print(F("Gyro bias: "));
    DEBUG_SERIAL.print(gyroZero[ROLL]); DEBUG_SERIAL.print(F(" "));
    DEBUG_SERIAL.print(gyroZero[PITCH]); DEBUG_SERIAL.print(F(" "));
    DEBUG_SERIAL.println(gyroZero[YAW]);
}

void calibrateBarometer() {
    float sumPressure = 0;
    DEBUG_SERIAL.println(F("Calibrating barometer..."));
    
    for (int i = 0; i < BARO_CALIBRATION_SAMPLES; i++) {
        sumPressure += bmp.readPressure();
        delay(20);
    }
    
    baselinePressure = sumPressure / BARO_CALIBRATION_SAMPLES;
    EstAlt = 0;
    DEBUG_SERIAL.print(F("Baseline: ")); DEBUG_SERIAL.println(baselinePressure);
}

void readIMU() {
    float ax, ay, az, gx, gy, gz;
    
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        accADC[ROLL] = (int16_t)(ay * 8192);   // Scale to ~16-bit range
        accADC[PITCH] = (int16_t)(-ax * 8192);
        accADC[YAW] = (int16_t)(az * 8192);
    }
    
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gx, gy, gz);
        // Convert to raw values and remove bias
        gyroADC[ROLL] = (int16_t)(gx * GYRO_SCALE) - gyroZero[ROLL];
        gyroADC[PITCH] = (int16_t)(gy * GYRO_SCALE) - gyroZero[PITCH];
        gyroADC[YAW] = (int16_t)(gz * GYRO_SCALE) - gyroZero[YAW];
    }
    
    // Check for NaN/invalid
    if (isnan(gx) || isnan(ax)) {
        imuHealthy = false;
    } else {
        imuHealthy = true;
    }
}

// Simplified attitude estimation (complementary filter)
void computeIMU() {
    static uint32_t lastTime = 0;
    uint32_t now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    if (dt > 0.5f) dt = 0.004f;  // Guard
    lastTime = now;
    
    // Accelerometer angles (in 0.1 degrees)
    float accRoll = atan2(accADC[ROLL], sqrt((int32_t)accADC[PITCH]*accADC[PITCH] + (int32_t)accADC[YAW]*accADC[YAW])) * 573.0f;
    float accPitch = atan2(-accADC[PITCH], sqrt((int32_t)accADC[ROLL]*accADC[ROLL] + (int32_t)accADC[YAW]*accADC[YAW])) * 573.0f;
    
    // Gyro integration (gyroADC is already scaled)
    float gyroRollRate = gyroADC[ROLL] / GYRO_SCALE;  // deg/s
    float gyroPitchRate = gyroADC[PITCH] / GYRO_SCALE;
    
    // Complementary filter: 98% gyro, 2% accelerometer
    angle[ROLL] = (int16_t)(0.98f * (angle[ROLL] + gyroRollRate * dt * 10.0f) + 0.02f * accRoll);
    angle[PITCH] = (int16_t)(0.98f * (angle[PITCH] + gyroPitchRate * dt * 10.0f) + 0.02f * accPitch);
}

void readBarometer() {
    static uint32_t lastBaroTime = 0;
    static int32_t lastEstAlt = 0;
    uint32_t now = millis();
    
    if (now - lastBaroTime < 25) return;  // 40Hz max
    
    float pressure = bmp.readPressure();
    
    if (isnan(pressure)) {
        barometerHealthy = false;
        return;
    }
    barometerHealthy = true;
    
    // Calculate altitude in cm
    float altMeters = 44330.0f * (1.0f - pow(pressure / baselinePressure, 1.0f / 5.255f));
    int32_t newAlt = (int32_t)(altMeters * 100.0f);
    
    // Simple low-pass filter
    EstAlt = EstAlt + (newAlt - EstAlt) / 4;
    
    // Vertical speed (cm/s)
    float dtBaro = (now - lastBaroTime) / 1000.0f;
    vario = (EstAlt - lastEstAlt) / dtBaro;
    
    lastEstAlt = EstAlt;
    lastBaroTime = now;
}

void disarmMotors() {
    motorFL.writeMicroseconds(ESC_MIN_PULSE);
    motorFR.writeMicroseconds(ESC_MIN_PULSE);
    motorRL.writeMicroseconds(ESC_MIN_PULSE);
    motorRR.writeMicroseconds(ESC_MIN_PULSE);
    
    // Reset all PID integrators (as MultiWii does)
    errorGyroI[ROLL] = 0;
    errorGyroI[PITCH] = 0;
    errorGyroI[YAW] = 0;
    errorAngleI[ROLL] = 0;
    errorAngleI[PITCH] = 0;
    errorAltitudeI = 0;
    
    rcCommand[THROTTLE] = ESC_MIN_PULSE;
}

void emergencyDisarm(const char* reason) {
    armed = false;
    disarmMotors();
    DEBUG_SERIAL.print(F("EMERGENCY: "));
    DEBUG_SERIAL.println(reason);
    
    StaticJsonDocument<128> doc;
    doc["status"] = "emergency";
    doc["msg"] = reason;
    serializeJson(doc, FLIGHT_SERIAL);
    FLIGHT_SERIAL.println();
}

bool checkPreArmSafety() {
    if (!imuHealthy) {
        DEBUG_SERIAL.println(F("FAIL: IMU"));
        return false;
    }
    // Check level (angle in 0.1 deg, so 100 = 10 degrees)
    if (abs(angle[ROLL]) > 100 || abs(angle[PITCH]) > 100) {
        DEBUG_SERIAL.println(F("FAIL: Not level"));
        return false;
    }
    if (rcCommand[THROTTLE] > ESC_ARM_PULSE + 50) {
        DEBUG_SERIAL.println(F("FAIL: Throttle high"));
        return false;
    }
    DEBUG_SERIAL.println(F("Pre-arm OK"));
    return true;
}

// =========================================================
// ================= COMMAND PROCESSING ====================
// =========================================================
// JSON-based commands from Flask server:
//   {"cmd":"arm"}
//   {"cmd":"disarm"}
//   {"cmd":"rc","t":1500,"r":0,"p":0,"y":0}
//   {"cmd":"status"}
//   {"cmd":"takeoff","alt":1.0}
//   {"cmd":"land"}

void sendJsonResponse(const char* status, const char* message) {
    StaticJsonDocument<128> resp;
    resp["status"] = status;
    resp["msg"] = message;
    serializeJson(resp, FLIGHT_SERIAL);
    FLIGHT_SERIAL.println();
    
    // Also send to debug serial
    serializeJson(resp, DEBUG_SERIAL);
    DEBUG_SERIAL.println();
}

void processJsonCommand(const char* json) {
    lastCommandTime = millis();
    
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, json);
    
    if (error) {
        DEBUG_SERIAL.print(F("[DEBUG] JSON parse error: "));
        DEBUG_SERIAL.println(error.c_str());
        DEBUG_SERIAL.print(F("[DEBUG] Raw input: "));
        DEBUG_SERIAL.println(json);
        sendJsonResponse("error", "JSON parse failed");
        return;
    }
    
    const char* cmd = doc["cmd"];
    if (!cmd) {
        DEBUG_SERIAL.println(F("[DEBUG] No 'cmd' field in JSON"));
        sendJsonResponse("error", "No cmd field");
        return;
    }
    
    DEBUG_SERIAL.print(F("[DEBUG] Received command: "));
    DEBUG_SERIAL.println(cmd);
    
    // ========== ARM ==========
    if (strcmp(cmd, "arm") == 0) {
        if (!armed && checkPreArmSafety()) {
            armed = true;
            DEBUG_SERIAL.println(F("✅ Motors ARMED - BE CAREFUL!"));
            sendJsonResponse("ok", "Armed");
        } else if (armed) {
            sendJsonResponse("ok", "Already armed");
        } else {
            DEBUG_SERIAL.println(F("❌ Pre-arm checks FAILED"));
            sendJsonResponse("error", "Pre-arm check failed");
        }
        return;
    }
    
    // ========== DISARM ==========
    if (strcmp(cmd, "disarm") == 0) {
        armed = false;
        disarmMotors();
        DEBUG_SERIAL.println(F("🔴 Motors DISARMED"));
        sendJsonResponse("ok", "Disarmed");
        return;
    }
    
    // ========== STATUS ==========
    if (strcmp(cmd, "status") == 0) {
        StaticJsonDocument<256> resp;
        resp["status"] = "ok";
        resp["armed"] = armed;
        resp["mode"] = currentMode;
        resp["roll"] = angle[ROLL] / 10.0;
        resp["pitch"] = angle[PITCH] / 10.0;
        resp["alt"] = EstAlt;
        resp["throttle"] = rcCommand[THROTTLE];
        serializeJson(resp, FLIGHT_SERIAL);
        FLIGHT_SERIAL.println();
        return;
    }
    
    // ========== RC INPUT (NippleJS joystick) ==========
    // Format: {"cmd":"rc","t":1500,"r":0,"p":0,"y":0}
    if (strcmp(cmd, "rc") == 0) {
        // Throttle
        if (doc.containsKey("t")) {
            int thr = doc["t"] | MIN_THROTTLE;
            rcCommand[THROTTLE] = constrain(thr, MIN_THROTTLE, MAX_THROTTLE);
        }
        
        // Roll (from right joystick X axis) - convert degrees to rcCommand
        if (doc.containsKey("r")) {
            float roll = doc["r"] | 0.0f;
            rcCommand[ROLL] = constrain((int)(roll * 16.67f), -500, 500);  // ±30 deg -> ±500
        }
        
        // Pitch (from right joystick Y axis)
        if (doc.containsKey("p")) {
            float pitch = doc["p"] | 0.0f;
            rcCommand[PITCH] = constrain((int)(pitch * 16.67f), -500, 500);
        }
        
        // Yaw rate (from left joystick X axis)
        if (doc.containsKey("y")) {
            float yaw = doc["y"] | 0.0f;
            rcCommand[YAW] = constrain((int)(yaw * 2.78f), -500, 500);  // ±180 deg -> ±500
        }
        
        // No response for RC commands - they come at high frequency (20Hz)
        return;
    }
    
    // ========== TAKEOFF ==========
    if (strcmp(cmd, "takeoff") == 0) {
        if (!armed) {
            sendJsonResponse("error", "Not armed");
            return;
        }
        float alt = doc["alt"] | 1.0f;
        AltHold = constrain((int32_t)(alt * 100), 50, 1000);  // Convert m to cm
        currentMode = MODE_ALT_HOLD;
        DEBUG_SERIAL.print(F("[DEBUG] Takeoff to altitude: "));
        DEBUG_SERIAL.println(alt);
        sendJsonResponse("ok", "Takeoff started");
        return;
    }
    
    // ========== LAND ==========
    if (strcmp(cmd, "land") == 0) {
        currentMode = MODE_LAND;
        DEBUG_SERIAL.println(F("[DEBUG] Landing initiated"));
        sendJsonResponse("ok", "Landing started");
        return;
    }
    
    // ========== Unknown command ==========
    DEBUG_SERIAL.print(F("[DEBUG] Unknown command: "));
    DEBUG_SERIAL.println(cmd);
    sendJsonResponse("error", "Unknown command");
}

void processSerial() {
    static char buffer[256];
    static int bufIdx = 0;
    
    // Read from UART (Raspberry Pi)
    while (FLIGHT_SERIAL.available()) {
        char c = FLIGHT_SERIAL.read();
        if (c == '\n' || c == '\r') {
            if (bufIdx > 0) {
                buffer[bufIdx] = '\0';
                DEBUG_SERIAL.print(F("[DEBUG] UART RX: "));
                DEBUG_SERIAL.println(buffer);
                processJsonCommand(buffer);
                bufIdx = 0;
            }
        } else if (bufIdx < 255) {
            buffer[bufIdx++] = c;
        }
    }
    
    // Also read from USB for debugging (supports both JSON and simple commands)
    while (DEBUG_SERIAL.available()) {
        char c = DEBUG_SERIAL.read();
        if (c == '\n' || c == '\r') {
            if (bufIdx > 0) {
                buffer[bufIdx] = '\0';
                
                // Check if it's JSON (starts with {)
                if (buffer[0] == '{') {
                    processJsonCommand(buffer);
                } else {
                    // Simple debug commands: a=arm, d=disarm, s=status
                    if (buffer[0] == 'a') {
                        processJsonCommand("{\"cmd\":\"arm\"}");
                    } else if (buffer[0] == 'd') {
                        processJsonCommand("{\"cmd\":\"disarm\"}");
                    } else if (buffer[0] == 's') {
                        processJsonCommand("{\"cmd\":\"status\"}");
                    } else {
                        DEBUG_SERIAL.print(F("[DEBUG] Unknown input: "));
                        DEBUG_SERIAL.println(buffer);
                    }
                }
                bufIdx = 0;
            }
        } else if (bufIdx < 255) {
            buffer[bufIdx++] = c;
        }
    }
}

// =========================================================
// ================= PID CONTROLLER ========================
// =========================================================
// This is the MultiWii PID Controller #1 (evolved oldschool)
// Reference: MultiWii.cpp lines 1398-1460

void computePID() {
    int16_t rc;
    int16_t error, errorAngle;
    int16_t PTerm, ITerm, DTerm;
    int16_t PTermACC, ITermACC;
    int16_t delta;
    uint8_t axis;
    
    // PITCH & ROLL
    for (axis = 0; axis < 2; axis++) {
        rc = rcCommand[axis] << 1;  // Double the RC command
        error = rc - gyroADC[axis];
        
        // Integral with windup protection
        errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, 16000);
        
        // Anti-windup: reset I when gyro rate is high
        if (abs(gyroADC[axis]) > 640) errorGyroI[axis] = 0;
        
        // I term
        ITerm = (errorGyroI[axis] >> 7) * pid[axis].I8 >> 6;
        
        // P term from RC command
        PTerm = mul(rc, pid[axis].P8) >> 6;
        
        // ANGLE MODE (self-level) - use accelerometer to level
        if (angleMode) {
            // Limit to ±500 (50 degrees)
            errorAngle = constrain(rc, -500, 500) - angle[axis];
            
            // Angle error integral
            errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, 10000);
            
            // P from angle error
            PTermACC = mul(errorAngle, pid[3].P8) >> 7;
            
            // Limit PTermACC
            int16_t limit = pid[3].D8 * 5;
            PTermACC = constrain(PTermACC, -limit, limit);
            
            // I from angle error
            ITermACC = mul(errorAngleI[axis], pid[3].I8) >> 12;
            
            // Replace gyro PID with angle PID
            ITerm = ITermACC;
            PTerm = PTermACC;
        }
        
        // Subtract gyro rate * dynP8 for additional damping
        PTerm -= mul(gyroADC[axis], pid[axis].P8) >> 6;
        
        // D term with 3-sample moving average for noise reduction
        delta = gyroADC[axis] - lastGyro[axis];
        lastGyro[axis] = gyroADC[axis];
        
        DTerm = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        
        DTerm = mul(DTerm, pid[axis].D8) >> 5;
        
        // Final PID output for this axis
        axisPID[axis] = PTerm + ITerm - DTerm;
    }
    
    // YAW
    #define GYRO_P_MAX 300
    #define GYRO_I_MAX 250
    
    rc = rcCommand[YAW];
    error = rc - gyroADC[YAW];
    
    errorGyroI[YAW] += mul(error, pid[YAW].I8);
    errorGyroI[YAW] = constrain(errorGyroI[YAW], -((int32_t)1 << 23), ((int32_t)1 << 23));
    
    // Reset I when stick moved
    if (abs(rc) > 50) errorGyroI[YAW] = 0;
    
    PTerm = mul(error, pid[YAW].P8) >> 6;
    int16_t limitYaw = GYRO_P_MAX - pid[YAW].D8;
    PTerm = constrain(PTerm, -limitYaw, limitYaw);
    
    ITerm = constrain((int16_t)(errorGyroI[YAW] >> 13), -GYRO_I_MAX, GYRO_I_MAX);
    
    axisPID[YAW] = PTerm + ITerm;
}

// =========================================================
// ================= MOTOR MIXING ==========================
// =========================================================
// Quadcopter X configuration
// FL(CW)   FR(CCW)
//    \     /
//     \   /
//      [X]
//     /   \
//    /     \
// RL(CCW)  RR(CW)

void mixTable() {
    int16_t maxMotor;
    int16_t motor[4];
    
    motor[0] = rcCommand[THROTTLE] - axisPID[PITCH] + axisPID[ROLL] + axisPID[YAW];  // FL - CW
    motor[1] = rcCommand[THROTTLE] - axisPID[PITCH] - axisPID[ROLL] - axisPID[YAW];  // FR - CCW
    motor[2] = rcCommand[THROTTLE] + axisPID[PITCH] + axisPID[ROLL] - axisPID[YAW];  // RL - CCW
    motor[3] = rcCommand[THROTTLE] + axisPID[PITCH] - axisPID[ROLL] + axisPID[YAW];  // RR - CW
    
    // Find max motor value
    maxMotor = motor[0];
    for (int i = 1; i < 4; i++) {
        if (motor[i] > maxMotor) maxMotor = motor[i];
    }
    
    // Scale down if any motor exceeds max
    if (maxMotor > ESC_MAX_PULSE) {
        for (int i = 0; i < 4; i++) {
            motor[i] -= maxMotor - ESC_MAX_PULSE;
        }
    }
    
    // Constrain to ESC range
    for (int i = 0; i < 4; i++) {
        motor[i] = constrain(motor[i], ESC_MIN_PULSE, ESC_MAX_PULSE);
    }
    
    // Write to motors
    if (armed) {
        motorFL.writeMicroseconds(motor[0]);
        motorFR.writeMicroseconds(motor[1]);
        motorRL.writeMicroseconds(motor[2]);
        motorRR.writeMicroseconds(motor[3]);
    } else {
        disarmMotors();
    }
}

// =========================================================
// ================= SETUP =================================
// =========================================================

void setup() {
    DEBUG_SERIAL.begin(BAUD_RATE);
    delay(100);
    FLIGHT_SERIAL.begin(BAUD_RATE);
    
    DEBUG_SERIAL.println(F("\n================================"));
    DEBUG_SERIAL.println(F("  DRONE FC - MultiWii PID v1.0"));
    DEBUG_SERIAL.println(F("================================\n"));
    
    // Initialize PID gains
    initPIDGains();
    
    // Initialize IMU
    if (!IMU.begin()) {
        DEBUG_SERIAL.println(F("ERROR: IMU failed!"));
        while (1) delay(100);
    }
    DEBUG_SERIAL.println(F("IMU OK"));
    
    calibrateGyro();
    
    // Initialize barometer
    if (!bmp.begin()) {
        DEBUG_SERIAL.println(F("WARNING: BMP180 not found"));
        barometerHealthy = false;
    } else {
        DEBUG_SERIAL.println(F("BMP180 OK"));
        calibrateBarometer();
        barometerHealthy = true;
    }
    
    // Initialize motors
    motorFL.attach(MOTOR_FL_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    motorFR.attach(MOTOR_FR_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    motorRL.attach(MOTOR_RL_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    motorRR.attach(MOTOR_RR_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    
    disarmMotors();
    DEBUG_SERIAL.println(F("Motors initialized"));
    
    delay(2000);
    
    DEBUG_SERIAL.println(F("\nReady! Waiting for commands..."));
    previousTime = micros();
}

// =========================================================
// ================= MAIN LOOP =============================
// =========================================================

void loop() {
    currentTime = micros();
    cycleTime = currentTime - previousTime;
    
    // Maintain loop rate
    #if defined(LOOP_TIME)
    if (cycleTime < LOOP_TIME) return;
    #endif
    previousTime = currentTime;
    
    // Failsafe
    if (armed && (millis() - lastCommandTime > 1000)) {
        emergencyDisarm("Failsafe");
    }
    
    // Process commands
    processSerial();
    
    // Read sensors
    readIMU();
    
    if (!imuHealthy && armed) {
        emergencyDisarm("IMU failure");
        return;
    }
    
    // Compute attitude
    computeIMU();
    
    // Read barometer
    readBarometer();
    
    // Emergency angle check (600 = 60 degrees)
    if (armed && (abs(angle[ROLL]) > 600 || abs(angle[PITCH]) > 600)) {
        emergencyDisarm("Extreme angle");
        return;
    }
    
    // Reset I-term when throttle is low
    if (rcCommand[THROTTLE] < 1050) {
        errorGyroI[ROLL] = 0;
        errorGyroI[PITCH] = 0;
        errorGyroI[YAW] = 0;
        errorAngleI[ROLL] = 0;
        errorAngleI[PITCH] = 0;
    }
    
    // Compute PID
    computePID();
    
    // Altitude hold (if enabled)
    if (currentMode == MODE_ALT_HOLD && barometerHealthy) {
        int16_t altError = AltHold - EstAlt;
        errorAltitudeI = constrain(errorAltitudeI + altError / 16, -1000, 1000);
        BaroPID = constrain(pid[3].P8 * altError / 128 + pid[3].I8 * errorAltitudeI / 64, -200, 200);
        rcCommand[THROTTLE] = constrain(rcCommand[THROTTLE] + BaroPID, MIN_THROTTLE, MAX_THROTTLE);
    }
    
    // Mix motors
    mixTable();
    
    // Send telemetry at 10Hz - JSON format for Flask server
    if (millis() - lastTelemetryTime > 100) {
        StaticJsonDocument<256> telem;
        telem["type"] = "telemetry";
        telem["roll"] = angle[ROLL];       // 0.1 degree units
        telem["pitch"] = angle[PITCH];     // 0.1 degree units
        telem["yaw"] = (int)(gyroADC[YAW] / GYRO_SCALE);  // Yaw rate
        telem["thr"] = rcCommand[THROTTLE];
        telem["alt"] = EstAlt;             // cm
        telem["armed"] = armed;
        telem["mode"] = currentMode;
        
        serializeJson(telem, FLIGHT_SERIAL);
        FLIGHT_SERIAL.println();
        
        lastTelemetryTime = millis();
    }
}
