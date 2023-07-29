#include "hBridgeDriver.h"

Hbridge::Hbridge(uint8_t pin1H, uint8_t pin1L, uint8_t pin2H, uint8_t pin2L, float maxDutyCycle, uint32_t pwmFreq, uint8_t deadTime)
{
    m_maxDutyCycle = maxDutyCycle;
    m_pwmFreq = pwmFreq;
    m_deadTime = deadTime;

    mcpwm_config_t pwm_config;
    pwm_config.frequency = m_pwmFreq;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config));
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config));

    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, pin1H));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, pin1L));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, pin2H));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, pin2L));
    ESP_ERROR_CHECK(mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, m_deadTime, m_deadTime));
    ESP_ERROR_CHECK(mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, m_deadTime, m_deadTime));
    m_deadTimeEnabled0 = true;
    m_deadTimeEnabled1 = true;
    ESP_ERROR_CHECK(mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0));
    ESP_ERROR_CHECK(mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1));
    ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 0));
    ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 0));
}

void Hbridge::enableDeadTime()
{
    if (!m_deadTimeEnabled0) {
        ESP_ERROR_CHECK(mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, m_deadTime, m_deadTime));
        m_deadTimeEnabled0 = true;
    }
    if (!m_deadTimeEnabled1) {
        ESP_ERROR_CHECK(mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, m_deadTime, m_deadTime));
        m_deadTimeEnabled1 = true;
    }
}

void Hbridge::drive(float dutyCycle, bool reverseDirection)
{
    m_reverseDirection = reverseDirection;
    Hbridge::enableDeadTime();
    ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT_0, m_reverseDirection ? MCPWM_TIMER_0 : MCPWM_TIMER_1, MCPWM_GEN_A, 0));
    ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT_0, m_reverseDirection ? MCPWM_TIMER_1 : MCPWM_TIMER_0, MCPWM_GEN_A, min(dutyCycle, m_maxDutyCycle)));
}

void Hbridge::brake()
{
    Hbridge::enableDeadTime();
    ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 0));
    ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 0));
}

void Hbridge::coast()
{
    ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 0));
    ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 0));
    if (m_reverseDirection) { // TIMER0 was negative side, let it float high with indcutive spike so solenoid decays quickly
        ESP_ERROR_CHECK(mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_0));
        m_deadTimeEnabled0 = false;
    } else {
        ESP_ERROR_CHECK(mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_1));
        m_deadTimeEnabled1 = false;
    }
}
