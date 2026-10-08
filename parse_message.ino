#pragma once

#include "pins_constants.h"

// Exact UART frame: RG,FW,00.0,LF,BK,00.0\n (22 characters).
// FW is positive cm/s; BK is negative cm/s. The current Motor class
// stops on negative requests until reverse control is implemented.
// Return false without changing either motor if any part is malformed.
/*
bool parse_message(const String& message) {
    if (message.length() != 22 || message[21] != '\n' ||
        message[0] != 'R' || message[1] != 'G' ||
        message[11] != 'L' || message[12] != 'F' ||
        message[2] != ',' || message[5] != ',' ||
        message[10] != ',' || message[13] != ',' || message[16] != ',') {
        return false;
    }

    const bool rightForward = message[3] == 'F' && message[4] == 'W';
    const bool rightBackward = message[3] == 'B' && message[4] == 'K';
    const bool leftForward = message[14] == 'F' && message[15] == 'W';
    const bool leftBackward = message[14] == 'B' && message[15] == 'K';
    if ((!rightForward && !rightBackward) || (!leftForward && !leftBackward)) {
        return false;
    }

    const uint8_t digitPositions[] = {6, 7, 9, 17, 18, 20};
    for (uint8_t position : digitPositions) {
        if (message[position] < '0' || message[position] > '9') {
            return false;
        }
    }
    if (message[8] != '.' || message[19] != '.') {
        return false;
    }

    // Convert only after the entire frame has passed validation.
    // Subtracting '0' converts a digit character into its numeric value. For example, '3' - '0' equals 3 because their ASCII codes differ by three.
    float rightSpeed = (message[6] - '0') * 10.0f +
                      (message[7] - '0') + (message[9] - '0') / 10.0f;
    float leftSpeed = (message[17] - '0') * 10.0f +
                     (message[18] - '0') + (message[20] - '0') / 10.0f;
    rightMotor.setNewGoalSpeed(rightForward ? rightSpeed : -rightSpeed);
    leftMotor.setNewGoalSpeed(leftForward ? leftSpeed : -leftSpeed);
    return true;
}
*/

// Keep partial frames between loop calls without blocking motor updates.
// Discard oversized frames through their newline, then accept the next frame.
void readSerialMessages() {
    // only save 23 because that is the number of characters + \n to make above formatting
    static char frame[23];
    static uint8_t length = 0;
    static bool discard = false;

    while (Serial.available() > 0) {
        // Because it returns an integer representation of the byte, you often need to cast it to a char if you
        // are expecting text. For example, if the computer sends the character 'A', Serial.read() will return its ASCII value 65
        const char received = static_cast<char>(Serial.read());
        if (!discard) {
            if (length < 22) {
                frame[length++] = received;
            } else {
                discard = true;
            }
        }
        if (received == '\n') {
            if (!discard) {
                frame[length] = '\0';
                Serial.println(frame);
                //parse_message(String(frame));
            }
            length = 0;
            discard = false;
        }
    }
}
