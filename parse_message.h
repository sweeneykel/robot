#pragma once

#include <Arduino.h>

struct MotorSpeeds {
    float right;
    float left;
    bool valid;
};

MotorSpeeds parse_message(const String& message);
