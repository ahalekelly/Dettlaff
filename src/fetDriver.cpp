#include "fetDriver.h"

Fet::Fet(uint8_t fetPin) {
        m_pin = fetPin;
        pinMode(m_pin, OUTPUT);
    }

void Fet::drive(float dutyCycle, bool reverseDirection) { // both parameters are ignored
    digitalWrite(m_pin, HIGH);
}

void Fet::coast() {
    digitalWrite(m_pin, LOW);
}

void Fet::brake() { // cannot actually brake with only 1 FET, coast instead
    digitalWrite(m_pin, LOW);
}