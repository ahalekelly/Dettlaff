#include "driver.h"
#include <Arduino.h>

class Drv : public Driver {
    uint8_t m_pin1;
    uint8_t m_pin2;

public:
    Drv(uint8_t pin1, uint8_t pin2, uint32_t pwmFrequency, bool coastHigh);
    void drive(float dutyCycle, bool reverseDirection);
    void brake();
    void coast();
};
