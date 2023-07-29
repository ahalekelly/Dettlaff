#include "fetDriver.h"

Fet::Fet(uint8_t fetPin)
{
    m_pin = fetPin;
    pinMode(m_pin, OUTPUT);
}

// both parameters are ignored
void Fet::drive(float dutyCycle, bool reverseDirection)
{
    digitalWrite(m_pin, HIGH);
}

void Fet::coast()
{
    digitalWrite(m_pin, LOW);
}

// cannot actually brake with only 1 FET, coast instead
void Fet::brake()
{
    digitalWrite(m_pin, LOW);
}