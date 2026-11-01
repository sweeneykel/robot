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
  // Retain partial lines without waiting for more serial input.
  static char commandBuffer[MAX_MSG_SIZE + 1] = {};
  static uint8_t commandLength = 0;
  static bool discardLine = false;
  static bool sawCarriageReturn = false;
  constexpr uint8_t SERIAL_BYTES_PER_LOOP = 16;

  for (uint8_t bytesRead = 0;
       bytesRead < SERIAL_BYTES_PER_LOOP && Serial.available() > 0;
       ++bytesRead) {
    const int incoming = Serial.read();
    if (incoming < 0) break;
    const char ch = static_cast<char>(incoming);

    if (ch == '\n') {
      if (!discardLine && commandLength == MAX_MSG_SIZE) {
        commandBuffer[commandLength] = '\0';
        MotorSpeeds parsedMsg = parse_message(String(commandBuffer));
        if (parsedMsg.valid) {
          rightMotor.setGoalRPM(parsedMsg.right);
          leftMotor.setGoalRPM(parsedMsg.left);
        }
      }
      commandLength = 0;
      discardLine = false;
      sawCarriageReturn = false;
      // Process at most one complete line before updating the motors.
      break;
    }

    if (discardLine) continue;
    // Accept CR only immediately before LF, never inside a command.
    if (sawCarriageReturn) {
      discardLine = true;
    } else if (ch == '\r') {
      sawCarriageReturn = true;
    } else if (ch == '\0' || commandLength >= MAX_MSG_SIZE) {
      // Discard the entire bad line; never interpret its tail as a command.
      discardLine = true;
    } else {
      commandBuffer[commandLength++] = ch;
    }
  }

  rightMotor.update();
  leftMotor.update();


  // only print every once in a while
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




