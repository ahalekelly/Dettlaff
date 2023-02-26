#include "at8870Driver.h"

At8870::At8870(uint8_t pin1, uint8_t pin2, uint32_t pwmFrequency) {
    m_pin1 = pin1;
    m_pin2 = pin2;
    pinMode(m_pin1, OUTPUT);
    digitalWrite(m_pin1, LOW);
    pinMode(m_pin2, OUTPUT);
    digitalWrite(m_pin2, LOW);
}

void At8870::drive(float dutyCycle, bool reverseDirection) { // dutyCycle is ignored - always runs full speed
    if (reverseDirection) {
        digitalWrite(m_pin1, LOW);
        digitalWrite(m_pin2, HIGH);
    } else {
        digitalWrite(m_pin1, HIGH);
        digitalWrite(m_pin2, LOW);
    }
}

void At8870::brake() {
    digitalWrite(m_pin1, HIGH);
    digitalWrite(m_pin2, HIGH);        
}

void At8870::coast() {
    digitalWrite(m_pin1, LOW);
    digitalWrite(m_pin2, LOW);        
}