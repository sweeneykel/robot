#include "parse_message.h"

// Payload: F00.0F00.0 (right motor first, left motor second).
// The UART reader removes the terminating newline before calling this function.
MotorSpeeds parse_message(const String& message) {
    if (message.length() != 10) {
        return {0.0f, 0.0f, false};
    }

    // Each motor has one direction character and a speed in d.dd format.
    for (uint8_t start = 0; start <= 5; start += 5) {
        if ((message[start] != 'F' && message[start] != 'B') ||
            message[start + 3] != '.' ||
            !isDigit(message[start + 1]) ||
            !isDigit(message[start + 2]) ||
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
