#include "Motor.h"
#include <math.h>

/**
 * Motor:: means “this belongs to the Motor class.” The :: is called the scope resolution operator.
 * Creates a motor and sets passed parameters as variables
 */
Motor::Motor(uint8_t in1, uint8_t in2,
      uint8_t encoderSpeedPin, uint8_t encoderDirPin,
      float gearRatio, uint16_t encoderPulsesPerRev,
      float wheelCircumferenceCm)
    // constructor’s member initializer list
    // initializes a member variable using a constructor argument which is necessary for const variables.
    : in1_(in1), in2_(in2),
      encoderSpeedPin_(encoderSpeedPin), encoderDirPin_(encoderDirPin),
      gearRatio_(gearRatio), encoderPulsesPerRev_(encoderPulsesPerRev),
      wheelCircumferenceCm_(wheelCircumferenceCm) {}


/**
 * Sets up motor including:
 * initializing input pins, stop motor, initializing interrupt pin, check validity of parameters,
 * set encoder speed and direction pins, sets up interrupt
 * @param encoderISR
 * @return true if all set-up steps were successful
 */
bool Motor::setUpMotor(void (*encoderISR)()) {
    if (initialized_) {
        // TODO: add more descriptive messages to help with debugging
        return false;
    }

    // initialize pins
    pinMode(in1_, OUTPUT);
    pinMode(in2_, OUTPUT);

    // ensure motor is stopped
    analogWrite(in1_, 0);
    analogWrite(in2_, 0);

    // takes a pin as an argument, and returns the same pin **IF** it can be used as an interrupt.
    const int interruptNumber = digitalPinToInterrupt(encoderSpeedPin_);

    // conditions to check for, returns false to setup if any condition is violated
    if (encoderISR == nullptr || interruptNumber == NOT_AN_INTERRUPT ||
        !isfinite(gearRatio_) || gearRatio_ <= 0.0f ||
        encoderPulsesPerRev_ == 0 ||
        !isfinite(wheelCircumferenceCm_) || wheelCircumferenceCm_ <= 0.0f) {
        // TODO: add more descriptive messages to help with debugging
        return false;
    }

    // set up outputs from motor to track encoder speed and direction
    pinMode(encoderSpeedPin_, INPUT_PULLUP);
    pinMode(encoderDirPin_, INPUT_PULLUP);

    // set up interrupt
    startTime_us_ = micros();
    initialized_ = true;
    attachInterrupt(interruptNumber, encoderISR, RISING);
    return true;
}

/**
 * Takes a cm/second speed and converts to RPM
 * @param speedCmPerSecond
 */
void Motor::setNewGoalSpeed(float speedCmPerSecond) {
    if (!initialized_ || !isfinite(speedCmPerSecond) || speedCmPerSecond <= 0.0f) {
        // TODO: add more descriptive messages to help with debugging
        stop();
        return;
    }
    const float newGoalRPM = speedCmPerSecond * 60.0f / wheelCircumferenceCm_;
    if (!isfinite(newGoalRPM)) {
        // TODO: add more descriptive messages to help with debugging
        stop();
        return;
    }
    goalRPM_ = newGoalRPM;
}

void Motor::update() {
    // if things are set up
    if (!initialized_) {
        // TODO: add more descriptive messages to help with debugging
        return;
    }
    calculateControlledVariable();
    const uint32_t now_us = micros();
    const uint32_t dt_us = now_us - startTime_us_;
    startTime_us_ = now_us;
    if (goalRPM_ <= 0.0f) {
        stop();
        return;
    }
    if (dt_us == 0) {
        return;
    }
    const float errorRPM = goalRPM_ - yRPM_;
    const float output = piController(errorRPM, dt_us / 1000000.0f);

    applyPWM(static_cast<int>(output));
}

void Motor::stop() {
    goalRPM_ = 0.0f;
    accumErrorIntegral_PWM = 0.0f;
    actuatingSignal_PWM_ = 0;
    if (initialized_) {
        applyPWM(0);
        startTime_us_ = micros();
    }
    // Retain encoder feedback: the wheel may still be coasting.
}

float Motor::getRPM() const {
    return yRPM_;
}

void Motor::setGains(float kp, float ki) {
    if (isfinite(kp) && isfinite(ki) && kp >= 0.0f && ki >= 0.0f) {
        kp_ = kp;
        ki_ = ki;
        accumErrorIntegral_PWM = 0.0f;
    }
}

void Motor::onEncoderPulse() {
    const uint32_t now_us = micros();
    const uint32_t elapsed_us = now_us - lastEncoderTime_us_;
    elapsedEncoderTime_us_ =
        (hasEncoderPulse_ && elapsed_us <= STOP_TIMEOUT_US) ? elapsed_us : 0;
    lastEncoderTime_us_ = now_us;
    hasEncoderPulse_ = true;
    // Direction decoding is deferred until reverse control is implemented.
}

void Motor::calculateControlledVariable() {
    // Call from the main loop with interrupts enabled. Volatile alone does
    // not prevent torn multi-byte reads; copy all ISR state together.
    noInterrupts();
    const uint32_t last_us = lastEncoderTime_us_;
    const uint32_t period_us = elapsedEncoderTime_us_;
    const bool hasPulse = hasEncoderPulse_;
    interrupts();

    if (!hasPulse || period_us == 0 || micros() - last_us > STOP_TIMEOUT_US) {
        yRPM_ = 0.0f;
    } else {
        // Convert before multiplying to avoid integer overflow.
        yRPM_ = 60000000.0f /
            (static_cast<float>(period_us) * encoderPulsesPerRev_ * gearRatio_);
    }
}

float Motor::piController(float errorRPM, float dtSeconds) {
    const float errorProportional_PWM = kp_ * errorRPM;

    // if there is a integral accumulation...
    if (ki_ > 0.0f) {
        // add new integral error to previous accumulated integral error, store in tmp
        const float candidateAccumErrorIntegral_PWM = accumErrorIntegral_PWM + (errorRPM * dtSeconds);
        const float candidateOutput_PWM = errorProportional_PWM + (ki_ * candidateAccumErrorIntegral_PWM);

        // Integrate if;
        // errorRPM = goalRPM_ - yRPM_;
        if ((candidateOutput_PWM >= 0.0f && candidateOutput_PWM <= 255.0f) ||  // 1. within range, no windup
            (candidateOutput_PWM > 255.0f && errorRPM < 0.0f) ||                // 2. overshoot & need to slow down, will eat away at accumulated integrl error
            (candidateOutput_PWM < 0.0f && errorRPM > 0.0f)) {                  // 3. need to speed up
            accumErrorIntegral_PWM = candidateAccumErrorIntegral_PWM;
        }
    }
    // Clamp before converting to an integer PWM command.
    return constrain(errorProportional_PWM + ki_ * accumErrorIntegral_PWM, 0.0f, 255.0f);
}

void Motor::applyPWM(int pwm) {
    actuatingSignal_PWM_ = constrain(pwm, 0, 255);
    analogWrite(in2_, 0);
    analogWrite(in1_, actuatingSignal_PWM_);
}
