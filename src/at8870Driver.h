#include "driver.h"
#include <Arduino.h>

class At8870 : public Driver {
    uint8_t m_pin1;
    uint8_t m_pin2;

public:
    At8870(uint8_t pin1, uint8_t pin2, uint32_t pwmFrequency);
    void drive(float dutyCycle, bool reverseDirection);
    void brake();
    void coast();
};
