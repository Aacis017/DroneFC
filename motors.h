/*
 * Motor Control for Quadcopter X Configuration
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <Servo.h>
#include "config.h"

class Motors {
public:
    void init() {
        // Attach ESCs to pins
        motorFL.attach(MOTOR_FL_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
        motorFR.attach(MOTOR_FR_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
        motorRL.attach(MOTOR_RL_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
        motorRR.attach(MOTOR_RR_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
        
        // Initialize to minimum (stopped)
        stop();
    }
    
    void arm() {
        // Send arming signal
        motorFL.writeMicroseconds(ESC_ARM_PULSE);
        motorFR.writeMicroseconds(ESC_ARM_PULSE);
        motorRL.writeMicroseconds(ESC_ARM_PULSE);
        motorRR.writeMicroseconds(ESC_ARM_PULSE);
    }
    
    void stop() {
        motorFL.writeMicroseconds(ESC_MIN_PULSE);
        motorFR.writeMicroseconds(ESC_MIN_PULSE);
        motorRL.writeMicroseconds(ESC_MIN_PULSE);
        motorRR.writeMicroseconds(ESC_MIN_PULSE);
    }
    
    void update(float throttle, float roll, float pitch, float yaw) {
        /*
         * Quadcopter X configuration motor mixing
         * 
         *     FL(CW)    FR(CCW)
         *        \     /
         *         \   /
         *          [X]
         *         /   \
         *        /     \
         *     RL(CCW)   RR(CW)
         * 
         * Roll: Right = positive, increases left motors
         * Pitch: Forward = positive, increases rear motors
         * Yaw: CW = positive, increases CCW motors (FR, RL)
         */
        
        // FL(CW) and RR(CW) spin same direction
        // FR(CCW) and RL(CCW) spin same direction
        // For CW yaw: decrease CW motors, increase CCW motors
        float fl = throttle + pitch + roll + yaw;   // CW motor
        float fr = throttle + pitch - roll - yaw;   // CCW motor
        float rl = throttle - pitch + roll - yaw;   // CCW motor
        float rr = throttle - pitch - roll + yaw;   // CW motor
        
        // Constrain to ESC range
        values.fl = constrain((int)fl, ESC_MIN_PULSE, ESC_MAX_PULSE);
        values.fr = constrain((int)fr, ESC_MIN_PULSE, ESC_MAX_PULSE);
        values.rl = constrain((int)rl, ESC_MIN_PULSE, ESC_MAX_PULSE);
        values.rr = constrain((int)rr, ESC_MIN_PULSE, ESC_MAX_PULSE);
        
        // Write to ESCs
        motorFL.writeMicroseconds(values.fl);
        motorFR.writeMicroseconds(values.fr);
        motorRL.writeMicroseconds(values.rl);
        motorRR.writeMicroseconds(values.rr);
    }
    
    MotorValues getValues() const {
        return values;
    }
    
private:
    Servo motorFL, motorFR, motorRL, motorRR;
    MotorValues values;
};

#endif // MOTORS_H
