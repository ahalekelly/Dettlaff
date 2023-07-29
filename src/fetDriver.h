#include "driver.h"
#include <Arduino.h>

class Fet : public Driver {
public:
    uint8_t m_pin;
    Fet(uint8_t fetPin);
    void drive(float dutyCycle, bool reverseDirection);
    void brake();
    void coast();
};