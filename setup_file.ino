#include "pins_constants.h"
#include "Motor.h"

constexpr uint32_t PRINT_INTERVAL_MS = 1000;  // 100 = 10 prints per second

//Initialize right motor
Motor rightMotor(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2,
                RIGHT_ENCODER_GREEN_SPEED, RIGHT_ENCODER_YELLOW_DIR,
                RIGHT_GEARING, RIGHT_ENCODERMULT, RIGHT_WHEEL_CIRCUMFERENCE_CM);

//Initialize left motor
Motor leftMotor(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2,
                LEFT_ENCODER_GREEN_SPEED, LEFT_ENCODER_YELLOW_DIR,
                LEFT_GEARING, LEFT_ENCODERMULT, LEFT_WHEEL_CIRCUMFERENCE_CM);

// TODO: UNDERSTAND THIS!
void rightEncoderISR() {
  rightMotor.onEncoderPulse();
}

void leftEncoderISR() {
  leftMotor.onEncoderPulse();
}

void setup() {
  Serial.begin(9600);
  rightMotor.setUpMotor(rightEncoderISR);
  rightMotor.stop();
  leftMotor.setUpMotor(leftEncoderISR);
  leftMotor.stop();
  delay(100);



  rightMotor.setNewGoalSpeed(2);
  rightMotor.setGains(3, 2);

  leftMotor.setNewGoalSpeed(5);
  leftMotor.setGains(3, 2);


}

void loop() {
  /*
  interrupt event to watch for: new goal speed set. Currently, will run entire loop and 
  only check if goalSpeed is updated once back at the beginning. TODO: create a type of 
  variable interrupt so that if a new speed is assigned, this is immediately implemented.
  okay for now because assuming that the loop runs very quickly.
  */

  rightMotor.update();
  leftMotor.update();




}
