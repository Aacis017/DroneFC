/*
 * Command Handler - UART Protocol from Flask Server
 * 
 * JSON-based commands from Raspberry Pi
 */

#ifndef COMMAND_H
#define COMMAND_H

#include <ArduinoJson.h>
#include "config.h"
#include "types.h"

// Forward declaration of motors for arming
extern Motors motors;

class CommandHandler {
public:
    void processSerial(Stream& serial, FlightState& state) {
        while (serial.available()) {
            char c = serial.read();
            
            if (c == '\n' || c == '\r') {
                if (bufferIndex > 0) {
                    buffer[bufferIndex] = '\0';
                    parseCommand(buffer, state, serial);
                    bufferIndex = 0;
                }
            } else if (bufferIndex < BUFFER_SIZE - 1) {
                buffer[bufferIndex++] = c;
            }
        }
    }
    
private:
    static const int BUFFER_SIZE = 256;
    char buffer[BUFFER_SIZE];
    int bufferIndex = 0;
    
    void parseCommand(const char* json, FlightState& state, Stream& serial) {
        StaticJsonDocument<256> doc;
        
        DeserializationError error = deserializeJson(doc, json);
        if (error) {
            sendResponse(serial, "error", "JSON parse failed");
            return;
        }
        
        const char* cmd = doc["cmd"];
        if (!cmd) {
            sendResponse(serial, "error", "No cmd field");
            return;
        }
        
        // ========== ARM / DISARM ==========
        if (strcmp(cmd, "arm") == 0) {
            if (canArm(state)) {
                state.armed = true;
                motors.arm();
                sendResponse(serial, "ok", "Armed");
            } else {
                sendResponse(serial, "error", "Cannot arm - check level/throttle");
            }
        }
        else if (strcmp(cmd, "disarm") == 0) {
            state.armed = false;
            motors.stop();
            state.throttle = ESC_MIN_PULSE;
            sendResponse(serial, "ok", "Disarmed");
        }
        
        // ========== THROTTLE ==========
        else if (strcmp(cmd, "throttle") == 0) {
            int val = doc["val"] | 0;
            state.throttle = constrain(val, MIN_THROTTLE, MAX_THROTTLE);
            sendResponse(serial, "ok", "Throttle set");
        }
        
        // ========== MOVEMENT ==========
        else if (strcmp(cmd, "move") == 0) {
            float roll = doc["roll"] | 0.0f;
            float pitch = doc["pitch"] | 0.0f;
            float yaw = doc["yaw"] | 0.0f;
            
            state.targetRoll = constrain(roll, -MAX_ROLL_ANGLE, MAX_ROLL_ANGLE);
            state.targetPitch = constrain(pitch, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE);
            state.targetYaw = constrain(yaw, -MAX_YAW_RATE, MAX_YAW_RATE);
            
            // Don't send response for move commands (high frequency)
        }
        
        // ========== RC INPUT (NippleJS joystick) ==========
        // Use this for continuous joystick control - no response sent (high frequency)
        // Format: {"cmd":"rc","t":1500,"r":0,"p":0,"y":0}
        // t = throttle (1000-2000), r = roll (-30 to 30), p = pitch, y = yaw rate
        else if (strcmp(cmd, "rc") == 0) {
            // Throttle
            if (doc.containsKey("t")) {
                int thr = doc["t"] | MIN_THROTTLE;
                state.throttle = constrain(thr, MIN_THROTTLE, MAX_THROTTLE);
            }
            
            // Roll (from right joystick X axis)
            if (doc.containsKey("r")) {
                float roll = doc["r"] | 0.0f;
                state.targetRoll = constrain(roll, -MAX_ROLL_ANGLE, MAX_ROLL_ANGLE);
            }
            
            // Pitch (from right joystick Y axis)
            if (doc.containsKey("p")) {
                float pitch = doc["p"] | 0.0f;
                state.targetPitch = constrain(pitch, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE);
            }
            
            // Yaw rate (from left joystick X axis)
            if (doc.containsKey("y")) {
                float yaw = doc["y"] | 0.0f;
                state.targetYaw = constrain(yaw, -MAX_YAW_RATE, MAX_YAW_RATE);
            }
            
            // No response for RC commands - they come at high frequency
        }
        
        // ========== MODE ==========
        else if (strcmp(cmd, "mode") == 0) {
            const char* mode = doc["mode"];
            if (mode) {
                if (strcmp(mode, "stabilize") == 0) {
                    state.mode = MODE_STABILIZE;
                } else if (strcmp(mode, "althold") == 0) {
                    state.mode = MODE_ALT_HOLD;
                }
                sendResponse(serial, "ok", "Mode changed");
            }
        }
        
        // ========== ALTITUDE (for alt hold) ==========
        else if (strcmp(cmd, "altitude") == 0) {
            float alt = doc["val"] | 0.0f;
            state.targetAltitude = alt;
            state.mode = MODE_ALT_HOLD;
            sendResponse(serial, "ok", "Target altitude set");
        }
        
        // ========== TAKEOFF ==========
        else if (strcmp(cmd, "takeoff") == 0) {
            if (!state.armed) {
                sendResponse(serial, "error", "Not armed");
                return;
            }
            float alt = doc["alt"] | 1.0f;  // Default 1 meter
            state.targetAltitude = constrain(alt, 0.5f, 10.0f);
            state.mode = MODE_TAKEOFF;
            state.takeoffInProgress = true;
            sendResponse(serial, "ok", "Takeoff started");
        }
        
        // ========== LAND ==========
        else if (strcmp(cmd, "land") == 0) {
            state.mode = MODE_LAND;
            state.landingInProgress = true;
            sendResponse(serial, "ok", "Landing started");
        }
        
        // ========== STATUS ==========
        else if (strcmp(cmd, "status") == 0) {
            // Send current state as response
            StaticJsonDocument<256> resp;
            resp["status"] = "ok";
            resp["armed"] = state.armed;
            resp["mode"] = state.mode;
            resp["throttle"] = state.throttle;
            serializeJson(resp, serial);
            serial.println();
        }
        
        else {
            sendResponse(serial, "error", "Unknown command");
        }
    }
    
    bool canArm(FlightState& state) {
        if (state.armed) return false;  // Already armed
        
        #if ARM_SAFETY_CHECK
        // Check if drone is level
        extern SensorData sensors;
        if (abs(sensors.roll) > MAX_ARM_ANGLE || 
            abs(sensors.pitch) > MAX_ARM_ANGLE) {
            return false;
        }
        #endif
        
        // Check throttle is at minimum
        if (state.throttle > ESC_ARM_PULSE + 50) {
            return false;
        }
        
        return true;
    }
    
    void sendResponse(Stream& serial, const char* status, const char* message) {
        StaticJsonDocument<128> resp;
        resp["status"] = status;
        resp["msg"] = message;
        serializeJson(resp, serial);
        serial.println();
    }
};

#endif // COMMAND_H
