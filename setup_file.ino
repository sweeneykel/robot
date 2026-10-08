#include "pins_constants.h"
#include "Motor.h"

constexpr uint32_t PRINT_INTERVAL_MS = 1000;  // 100 = 10 prints per second

Motor rightMotor(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2,
                 RIGHT_ENCODER_GREEN_SPEED, RIGHT_ENCODER_YELLOW_DIR,
                 RIGHT_GEARING, RIGHT_ENCODERMULT, RIGHT_WHEEL_CIRCUMFERENCE_CM);

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
  Serial.begin(115200);
  rightMotor.setUpMotor(rightEncoderISR);
  rightMotor.stop();
  leftMotor.setUpMotor(leftEncoderISR);
  leftMotor.stop();
  delay(100);

  /**
  rightMotor.setGoalRPM(6);
  Serial.println("right motor goalRPM is: ");
  Serial.println(rightMotor.getGoalRPM());
  rightMotor.setGains(3, 2);

  leftMotor.setGoalRPM(6);
  Serial.println("left motor goalRPM is: ");
  Serial.println(leftMotor.getGoalRPM());
  leftMotor.setGains(3, 2);
  */
}

void loop() {
  readSerialMessages();

  rightMotor.update();
  leftMotor.update();


/**
  static uint32_t lastPrint_ms = 0;
  uint32_t now_ms = millis();
  constexpr uint32_t PRINT_INTERVAL_MS = 10000;  // 100 = 10 prints per second

  if (now_ms - lastPrint_ms >= PRINT_INTERVAL_MS) {
    lastPrint_ms = now_ms;
    Serial.println("right motor RPM: ");
    Serial.println(rightMotor.getRPM());
    Serial.println("left motor RPM: ");
    Serial.println(leftMotor.getRPM());
  }
*/
}

