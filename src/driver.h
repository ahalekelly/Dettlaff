#pragma once
#include <Arduino.h>

class Driver {
public:
    virtual void drive(float dutyCycle, bool reverseDirection) = 0;
    virtual void brake() = 0;
    virtual void coast() = 0;
};