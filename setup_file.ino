#include "pins_constants.h"
#include "Motor.h"

constexpr uint32_t PRINT_INTERVAL_MS = 1000;  // 100 = 10 prints per second

//Initialize right motor
Motor rightMotor(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2,
                RIGHT_ENCODER_GREEN_SPEED, RIGHT_ENCODER_GREEN_SPEED,
                RIGHT_GEARING, RIGHT_ENCODERMULT, RIGHT_WHEEL_CIRCUMFERENCE_CM);

// TODO: UNDERSTAND THIS!
void rightEncoderISR() {
  rightMotor.onEncoderPulse();
}

// Initialize left motor

void setup() {
  Serial.begin(9600);
  rightMotor.setUpMotor(rightEncoderISR);
  delay(100);
}

void loop() {
  /*
  interrupt event to watch for: new goal speed set. Currently, will run entire loop and 
  only check if goalSpeed is updated once back at the beginning. TODO: create a type of 
  variable interrupt so that if a new speed is assigned, this is immediately implemented.
  okay for now because assuming that the loop runs very quickly.
  */

  // motor input
  rightMotor.applyPWM(255);

}
