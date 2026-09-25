#pragma once

/*
Configuration: Compile time items that are defined
*/

// write to motor via these connections
constexpr uint8_t RIGHT_MOTOR_IN1 = 5;
constexpr uint8_t RIGHT_MOTOR_IN2 = 6;
constexpr uint8_t LEFT_MOTOR_IN1 = 10;
constexpr uint8_t LEFT_MOTOR_IN2 = 9;

// read from motor via these connections
constexpr uint8_t RIGHT_ENCODER_GREEN_SPEED = 2; // interrupt only 2 or 3
constexpr uint8_t RIGHT_ENCODER_YELLOW_DIR = 11;
constexpr uint8_t LEFT_ENCODER_GREEN_SPEED = 3;  // interrupt only 2 or 3
constexpr uint8_t LEFT_ENCODER_YELLOW_DIR = 7;

// specific to a particular motor
constexpr int RIGHT_GEARING = 298;
constexpr int RIGHT_ENCODERMULT = 14; // counts per revolution
constexpr int LEFT_GEARING = 298;
constexpr int LEFT_ENCODERMULT = 14; // counts per revolution

// specific to a particular wheelset
constexpr float RIGHT_WHEEL_CIRCUMFERENCE_CM = 7.0;
constexpr float LEFT_WHEEL_CIRCUMFERENCE_CM = 7.0;
