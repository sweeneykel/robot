#pragma once

#include "pins_constants.h"
#include "Motor.h"
#include "parse_message.h"

constexpr uint32_t PRINT_INTERVAL_MS = 1000;  // 100 = 10 prints per second

Motor rightMotor(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2,
                 RIGHT_ENCODER_GREEN_SPEED, RIGHT_ENCODER_YELLOW_DIR,
                 RIGHT_GEARING, RIGHT_ENCODERMULT, RIGHT_WHEEL_CIRCUMFERENCE_CM);

Motor leftMotor(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2,
                LEFT_ENCODER_GREEN_SPEED, LEFT_ENCODER_YELLOW_DIR,
                LEFT_GEARING, LEFT_ENCODERMULT, LEFT_WHEEL_CIRCUMFERENCE_CM);

void rightEncoderISR() {
    rightMotor.onEncoderPulse();
}

void leftEncoderISR() {
    leftMotor.onEncoderPulse();
}

void setup() {
  Serial.begin(115200);
  // TODO: If either .setUpMotor returns false then show error
  rightMotor.setUpMotor(rightEncoderISR);
  leftMotor.setUpMotor(leftEncoderISR);
  delay(100);

  rightMotor.setGains(4, 2);
  leftMotor.setGains(4, 2);
}

void loop() 
{
  if (Serial.available() > 0) 
  {
        // It stops at '\n' and does not include it in the returned String. The newline is consumed from the serial buffer.
        String msg = Serial.readStringUntil('\n');

        MotorSpeeds parsedMsg = parse_message(msg);

        if (parsedMsg.valid)
        {
          Serial.println("passed parsing!");
          rightMotor.setGoalRPM(parsedMsg.right);
          leftMotor.setGoalRPM(parsedMsg.left);
          Serial.print("rightMotorGoal: ");
          Serial.println(rightMotor.getGoalRPM());
          Serial.print("leftMotorGoal: ");
          Serial.println(leftMotor.getGoalRPM());
        }
  }

  rightMotor.update();
  leftMotor.update();

  static uint32_t lastPrint_ms = 0;
  uint32_t now_ms = millis();
  constexpr uint32_t PRINT_INTERVAL_MS = 1000;  // 100 = 10 prints per second

  if (now_ms - lastPrint_ms >= PRINT_INTERVAL_MS) 
  {
    lastPrint_ms = now_ms;
    Serial.print("rightMotor RPM: ");
    Serial.print(rightMotor.getRPM());
    Serial.print(" | leftMotor RPM: ");
    Serial.print(leftMotor.getRPM());
    Serial.print(" | leftMotor RPM: ");
    Serial.print(leftMotor.getRPM());
  }
}




