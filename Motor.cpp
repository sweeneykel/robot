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
 * setup()
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
    previousControlUpdateTimestamp_us_ = micros();
    initialized_ = true;
    attachInterrupt(interruptNumber, encoderISR, RISING);
    return true;
}

/**
 * setup()
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

/**
 * loop()
 *
 */
void Motor::update() {
    // if things are set up
    if (!initialized_) {
        // TODO: add more descriptive messages to help with debugging
        return;
    }

    calculateControlledVariable();   // calculates yRPM

    
    const uint32_t currentControlUpdateTimestamp_us = micros();
    const uint32_t controlUpdateInterval_us = currentControlUpdateTimestamp_us - previousControlUpdateTimestamp_us_;
    previousControlUpdateTimestamp_us_ = currentControlUpdateTimestamp_us;
    if (goalRPM_ <= 0.0f) {
        stop();
        return;
    }
    if (controlUpdateInterval_us == 0) {
        return;
    }
    const float errorRPM = goalRPM_ - yRPM_;
    const float output = piController(errorRPM, controlUpdateInterval_us / 1000000.0f);

    applyPWM(static_cast<int>(output));
}

/**
 * helper function. stops motor and resets all telemetry to zero.
 */
void Motor::stop() {
    goalRPM_ = 0.0f;
    accumErrorIntegral_PWM = 0.0f;
    actuatingSignal_PWM_ = 0;
    if (initialized_) {
        applyPWM(0);
        previousControlUpdateTimestamp_us_ = micros();
    }
    // Retain encoder feedback: the wheel may still be coasting.
}

/**
 * simple getter, returns stored yRPM_ value
 * @return yRPM
 */
float Motor::getRPM() const {
    return yRPM_;
}

/**
 * simple setter, sets gains used in proportional control
 * @param kp, ki
 */
void Motor::setGains(float kp, float ki) {
    if (isfinite(kp) && isfinite(ki) && kp >= 0.0f && ki >= 0.0f) {
        kp_ = kp;
        ki_ = ki;
        accumErrorIntegral_PWM = 0.0f;
    }
}

/**
 * initialized in <loc>EncoderISR() function before setup and passed to <loc>.setUpMotor(<loc>EncoderISR);
 */
void Motor::onEncoderPulse() {
    const uint32_t currentEncoderPulseTimestamp_us = micros();
    const uint32_t encoderPulseInterval_us = currentEncoderPulseTimestamp_us - latestEncoderPulseTimestamp_us_;
    encoderPulseInterval_us_ =
        (hasEncoderPulse_ && encoderPulseInterval_us <= ENCODER_NO_PULSE_TIMEOUT_US) ? encoderPulseInterval_us : 0;
    latestEncoderPulseTimestamp_us_ = currentEncoderPulseTimestamp_us;
    hasEncoderPulse_ = true;
    // Direction decoding is deferred until reverse control is implemented.
}

void Motor::calculateControlledVariable() {
    // Call from the main loop with interrupts enabled. Volatile alone does
    // not prevent torn multi-byte reads; copy all ISR state together.
    noInterrupts();
    const uint32_t latestEncoderPulseTimestampSnapshot_us = latestEncoderPulseTimestamp_us_;
    const uint32_t encoderPulseIntervalSnapshot_us = encoderPulseInterval_us_;
    const bool hasPulse = hasEncoderPulse_;
    interrupts();

    const uint32_t encoderPulseAge_us =
        micros() - latestEncoderPulseTimestampSnapshot_us;

    // TODO: investigate if all 3 are necessary, in which cases one fails
    if (!hasPulse || encoderPulseIntervalSnapshot_us == 0 || encoderPulseAge_us > ENCODER_NO_PULSE_TIMEOUT_US) {
        yRPM_ = 0.0f;

    } else {
        // Convert before multiplying to avoid integer overflow.
        yRPM_ = 60000000.0f /
            (static_cast<float>(encoderPulseIntervalSnapshot_us) * encoderPulsesPerRev_ * gearRatio_);
    }
}

float Motor::piController(float errorRPM, float controlUpdateInterval_s) {
    const float errorProportional_PWM = kp_ * errorRPM;

    // if ki_ = 0, then integration is not enabled, skip this.
    if (ki_ > 0.0f) {
        // add new integral error to previous accumulated integral error, store in tmp
        const float candidateAccumErrorIntegral_PWM = accumErrorIntegral_PWM + (errorRPM * controlUpdateInterval_s);
        const float candidateOutput_PWM = errorProportional_PWM + (ki_ * candidateAccumErrorIntegral_PWM);

        /**
         * Only keep candidate AccumErrorIntegral_PWM if one of three conditions is met;
         * case 1: motor impulse is not saturated
         * case 2: motor impulse is saturated and yRPM > goalRPM        (neg #) = k_p * (neg #) + k_i * (neg #)
         * case 3: motor impulse is effectively 0 and yRPM < goalRPM    (pos #) = k_p * (pos #) + k_i * (pos #)
         *
         * More importantly are the cases that are excluded. For example, if the the motor impulse is already saturated and
         * yRPM < goalRPM, don't continue to keep track of this discrepancy, the motor is going at full max and
         * additional penalty though the accumErrorIntegral_PWM won't contribute to reaching the goal any quicker. If anything,
         * there will be a whiplash once yRPM = goalRPM because of the acucmErrorIntegral_PWM will have been accuring all the while.
         */
        if ((candidateOutput_PWM >= 0.0f && candidateOutput_PWM <= 255.0f) ||
            (candidateOutput_PWM > 255.0f && errorRPM < 0.0f) ||
            (candidateOutput_PWM < 0.0f && errorRPM > 0.0f)) {
            accumErrorIntegral_PWM = candidateAccumErrorIntegral_PWM;
        }
    }
    // Clamp before converting to an integer PWM command.
    return constrain(errorProportional_PWM + ki_ * accumErrorIntegral_PWM, 0.0f, 255.0f);
}

/**
 * sends PWM signal to motor through analogWrite(pin, pwm)
 * @param pwm that has not yet been constrained 0-255
 */
void Motor::applyPWM(int pwm) {
    actuatingSignal_PWM_ = constrain(pwm, 0, 255);
    analogWrite(in2_, 0);
    analogWrite(in1_, actuatingSignal_PWM_);
}
