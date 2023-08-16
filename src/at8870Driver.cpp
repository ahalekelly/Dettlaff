#include "at8870Driver.h"

const uint8_t ledc_channel = 0;
const uint32_t ledc_max_frequency = 24000;
const uint8_t ledc_resolution = 10;

const uint32_t ledc_max_value = pow(2, ledc_resolution) - 1;

At8870::At8870(uint8_t pin1, uint8_t pin2, uint32_t pwmFrequency)
{
    ledcSetup(1, min(ledc_max_frequency, pwmFrequency), ledc_resolution);
    ledcAttachPin(pin1, 1);
    ledcWrite(1, 0);

    ledcSetup(2, min(ledc_max_frequency, pwmFrequency), ledc_resolution);
    ledcAttachPin(pin2, 2);
    ledcWrite(2, 0);
}

void At8870::drive(float dutyCycle, bool reverseDirection) // dutyCycle is ignored - always runs full speed
{
    uint32_t m_dutyCycleInt = min(ledc_max_value, max((uint32_t)0, static_cast<uint32_t>(dutyCycle * ledc_max_value / 100)));
    if (reverseDirection) {
        ledcWrite(1, 0);
        ledcWrite(2, m_dutyCycleInt);
    } else {
        ledcWrite(1, m_dutyCycleInt);
        ledcWrite(2, 0);
    }
}

void At8870::brake() // brake and coast are swapped for the DRV8243
{
    ledcWrite(1, ledc_max_value); // TODO: check on scope that this actually is full on
    ledcWrite(2, ledc_max_value);
}

void At8870::coast()
{
    ledcWrite(1, 0);
    ledcWrite(2, 0);
}