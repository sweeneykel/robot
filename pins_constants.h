#pragma once

/*

Configuration: Compile time items that are defined

*/

// write to motor via these connections
constexpr uint8_t RIGHT_MOTOR_IN1 = 5;
constexpr uint8_t RIGHT_MOTOR_IN2 = 6;

constexpr uint8_t MOTOR_BIN1 = 9;
constexpr uint8_t MOTOR_BIN2 = 10;

// read from motor via these connections
constexpr uint8_t RIGHT_ENCODER_GREEN_SPEED = 2;
constexpr uint8_t RIGH_ENCODER_YELLOW_DIR = 11;

constexpr uint8_t ENCODER_GREEN_SPEED_B = 7;
constexpr uint8_t ENCODER_YELLOW_DIR_B = 8;

// specific to a particular motor
constexpr int RIGHT_GEARING = 298;
constexpr int RIGHT_ENCODERMULT = 14; // counts per revolution

// specific to a particular wheelset
constexpr float RIGHT_WHEEL_CIRCUMFERENCE_CM = 7.0;
