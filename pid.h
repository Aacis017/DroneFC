/*
 * PID Controller
 */

#ifndef PID_H
#define PID_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float maxOutput = 400.0f) 
        : Kp(kp), Ki(ki), Kd(kd), maxOut(maxOutput) {
        reset();
    }
    
    float compute(float setpoint, float input, float dt) {
        float error = setpoint - input;
        
        // Proportional
        float pTerm = Kp * error;
        
        // Integral with anti-windup
        integral += error * dt;
        integral = constrain(integral, -maxOut / Ki, maxOut / Ki);
        float iTerm = Ki * integral;
        
        // Derivative (on measurement to avoid derivative kick)
        float derivative = (input - lastInput) / dt;
        float dTerm = -Kd * derivative;
        
        lastInput = input;
        lastError = error;
        
        // Sum and constrain
        float output = pTerm + iTerm + dTerm;
        return constrain(output, -maxOut, maxOut);
    }
    
    void reset() {
        integral = 0;
        lastError = 0;
        lastInput = 0;
    }
    
    void setGains(float kp, float ki, float kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }
    
private:
    float Kp, Ki, Kd;
    float maxOut;
    float integral;
    float lastError;
    float lastInput;
};

#endif // PID_H
