#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <ESP32Encoder.h>
#include <Arduino.h>

class Motor {
public:
    Motor(int encA, int encB, int pwmFwd, int pwmRev,
          double Kp, double Ki, double Kd, double setpoint,
          int channelFwd, int channelRev, bool reverseEncoder)
        : ENCODER_A(encA), ENCODER_B(encB),
          MOTOR_PWM_FWD(pwmFwd), MOTOR_PWM_REV(pwmRev),
          Kp(Kp), Ki(Ki), Kd(Kd), setpoint(setpoint),
          pwmChannelFwd(channelFwd), pwmChannelRev(channelRev),
          reverseEncoder(reverseEncoder)
    {
        encoder.attachHalfQuad(ENCODER_A, ENCODER_B);
        encoder.setCount(0);

        pinMode(MOTOR_PWM_FWD, OUTPUT);
        pinMode(MOTOR_PWM_REV, OUTPUT);

        ledcSetup(pwmChannelFwd, 1000, 8);  // 1kHz, 8-bit
        ledcSetup(pwmChannelRev, 1000, 8);
        ledcAttachPin(MOTOR_PWM_FWD, pwmChannelFwd);
        ledcAttachPin(MOTOR_PWM_REV, pwmChannelRev);

        lastTime = millis();
    }

    double update() {
        unsigned long currentTime = millis();
        if (currentTime - lastTime < sampleTime) return speed;
    
        long counts = encoder.getCount();
        encoder.clearCount();
    
        if (reverseEncoder) counts = -counts;
    
        double elapsedTime = (currentTime - lastTime) / 1000.0;
        speed = (counts / elapsedTime) / 840.0 * 60.0;  // RPM
    
        error = setpoint - speed;
        integral += error * elapsedTime;
        double derivative = (error - error_prev1) / elapsedTime;
        output = Kp * error + Ki * integral + Kd * derivative;
        error_prev1 = error;
        lastTime = currentTime;
    
        int pwm = constrain(abs((int)output), 0, 255);
    
        if (output > 5) { // small dead zone
            ledcWrite(pwmChannelFwd, pwm);
            ledcWrite(pwmChannelRev, 0);
        } else if (output < -5) {
            ledcWrite(pwmChannelFwd, 0);
            ledcWrite(pwmChannelRev, pwm);
        } else {
            ledcWrite(pwmChannelFwd, 0);
            ledcWrite(pwmChannelRev, 0);
        }
    
        return speed;
    }
    
    void setSetpoint(double newSetpoint) {
        setpoint = newSetpoint;
    }
    
    void stop() {
        ledcWrite(pwmChannelFwd, 0);
        ledcWrite(pwmChannelRev, 0);
        setpoint = 0;
        integral = 0;
        error = 0;
        error_prev1 = 0;
    }

    void forceStop() {
        // Immediately stop the motor and reset all control variables
        ledcWrite(pwmChannelFwd, 0);
        ledcWrite(pwmChannelRev, 0);
        setpoint = 0;
        integral = 0;
        error = 0;
        error_prev1 = 0;
        output = 0;
        speed = 0;
        encoder.clearCount();
    }
    
    double getSetpoint() const {
        return setpoint;
    }

    void resetEncoder() {
        encoder.clearCount();
    }

    long getEncoderCount() {
        return encoder.getCount();
    }

private:
    int ENCODER_A, ENCODER_B;
    int MOTOR_PWM_FWD, MOTOR_PWM_REV;
    int pwmChannelFwd, pwmChannelRev;

    ESP32Encoder encoder;

    double Kp, Ki, Kd;
    double setpoint, speed = 0, error = 0, error_prev1 = 0;
    double output = 0, integral = 0;

    unsigned long lastTime = 0;
    const double sampleTime = 50; // ms

    bool reverseEncoder;
};

#endif
