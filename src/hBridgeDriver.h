#include <Arduino.h>
#include "driver.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

class Hbridge : public Driver {
    float m_maxDutyCycle;
    uint32_t m_pwmFreq;
    uint8_t m_deadTime;
    bool m_deadTimeEnabled0;
    bool m_deadTimeEnabled1;
    bool m_reverseDirection; // true = TIMER0 positive
    void enableDeadTime();
public:
    Hbridge(uint8_t pin1H, uint8_t pin1L, uint8_t pin2H, uint8_t pin2L, float maxDutyCycle, uint32_t pwmFreq, uint8_t deadTime);
    void drive(float dutyCycle, bool reverseDirection);
    void brake();
    void coast();
};