#line 1 "C:\\Users\\16084\\OneDrive\\Desktop\\desktop\\mylawnmower\\controller_plants\\parse_message.h"
#pragma once

#include <Arduino.h>

struct MotorSpeeds {
    float right;
    float left;
    bool valid;
};

MotorSpeeds parse_message(const String& message);
