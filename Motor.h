#pragma once

#include <Arduino.h>

class Motor {
public:
    Motor(uint8_t in1, uint8_t in2,
          uint8_t encoderSpeedPin, uint8_t encoderDirPin,
          float gearRatio, uint16_t encoderPulsesPerRev,
          float wheelCircumferenceCm);

    // Call once from setup(), passing a wrapper for this particular motor:
    // void encoderA_ISR() { motorA.onEncoderPulse(); }
    // motorA.setUpMotor(encoderA_ISR);
    bool setUpMotor(void (*encoderISR)());

    // Forward-only control. Zero, negative, or invalid requests stop the motor.
    void setNewGoalSpeed(float speedCmPerSecond);
    void update();
    void stop();
    float getRPM() const;
    void setGains(float kp, float ki);
    // Called only from this motor's interrupt wrapper.
    void onEncoderPulse();
    void applyPWM(int pwm);


private:
    void calculateControlledVariable();
    float piController(float errorRPM, float dtSeconds);

    // VARIABLES: private so that cannot be changed during process
    const uint8_t in1_;
    const uint8_t in2_;
    const uint8_t encoderSpeedPin_;
    const uint8_t encoderDirPin_;
    const float gearRatio_;
    // Rising edges per motor-shaft revolution, before the gearbox.
    const uint16_t encoderPulsesPerRev_;
    const float wheelCircumferenceCm_;
    static constexpr uint32_t STOP_TIMEOUT_US = 250000;

    bool initialized_ = false;
    float goalRPM_ = 0.0f;
    float yRPM_ = 0.0f;
    float accumErrorIntegral_PWM = 0.0f;
    uint32_t startTime_us_ = 0;
    int actuatingSignal_PWM_ = 0;
    float kp_ = 7.0f;
    float ki_ = 2.0f;

    volatile uint32_t lastEncoderTime_us_ = 0;
    volatile uint32_t elapsedEncoderTime_us_ = 0;
    volatile bool hasEncoderPulse_ = false;
};
