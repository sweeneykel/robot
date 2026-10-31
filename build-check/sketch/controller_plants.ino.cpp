#include <Arduino.h>
#line 17 "C:\\Users\\16084\\OneDrive\\Desktop\\desktop\\mylawnmower\\controller_plants\\setup_file.ino"
void rightEncoderISR();
#line 21 "C:\\Users\\16084\\OneDrive\\Desktop\\desktop\\mylawnmower\\controller_plants\\setup_file.ino"
void leftEncoderISR();
#line 25 "C:\\Users\\16084\\OneDrive\\Desktop\\desktop\\mylawnmower\\controller_plants\\setup_file.ino"
void setup();
#line 37 "C:\\Users\\16084\\OneDrive\\Desktop\\desktop\\mylawnmower\\controller_plants\\setup_file.ino"
void loop();
#line 0 "C:\\Users\\16084\\OneDrive\\Desktop\\desktop\\mylawnmower\\controller_plants\\setup_file.ino"
#line 1 "C:\\Users\\16084\\OneDrive\\Desktop\\desktop\\mylawnmower\\controller_plants\\simplified_controller_plant.ino"
#include "parse_message.h"

MotorSpeeds parse_message(const String& message);
#line 1 "C:\\Users\\16084\\OneDrive\\Desktop\\desktop\\mylawnmower\\controller_plants\\parse_message.ino"
#include "parse_message.h"

// Payload: F0.00F0.00 (right motor first, left motor second).
// The UART reader removes the terminating newline before calling this function.
MotorSpeeds parse_message(const String& message) {
    if (message.length() != 10) {
        return {0.0f, 0.0f, false};
    }

    // Each motor has one direction character and a speed in d.dd format.
    for (uint8_t start = 0; start <= 5; start += 5) {
        if ((message[start] != 'F' && message[start] != 'B') ||
            message[start + 2] != '.' ||
            !isDigit(message[start + 1]) ||
            !isDigit(message[start + 3]) ||
            !isDigit(message[start + 4])) {
            return {0.0f, 0.0f, false};
        }
    }

    float rightSpeed = message.substring(1, 5).toFloat();
    float leftSpeed = message.substring(6, 10).toFloat();
    if (message[0] == 'B') rightSpeed = -rightSpeed;
    if (message[5] == 'B') leftSpeed = -leftSpeed;

    return {rightSpeed, leftSpeed, true};
}

#line 1 "C:\\Users\\16084\\OneDrive\\Desktop\\desktop\\mylawnmower\\controller_plants\\setup_file.ino"
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
  rightMotor.setUpMotor(rightEncoderISR);
  leftMotor.setUpMotor(leftEncoderISR);
  delay(100);

  /**
  rightMotor.setGains(3, 2);
  leftMotor.setGains(3, 2);
  */
}

void loop() {
  if (Serial.available() > 0) {

        String msg = Serial.readStringUntil('\n');

        MotorSpeeds mtrCmmd = parse_message(msg);

        if (mtrCmmd.valid){
          Serial.print(mtrCmmd.right);
          Serial.print(mtrCmmd.left);
        }

      
        }

    rightMotor.update();
    leftMotor.update();

    }


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



