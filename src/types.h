#ifndef __types_h_
#define __types_h_
#include <Arduino.h>

enum flywheelState_t {
    STATE_IDLE,
    STATE_ACCELERATING, // ACCELERATING = wheels not yet at full speed
    STATE_FULLSPEED, // REV = wheels at full speed
};

enum pusherType_t {
    NO_PUSHER,
    PUSHER_MOTOR_CLOSEDLOOP,
    PUSHER_SOLENOID_OPENLOOP,
};

enum pusherDriverType_t {
    NO_DRIVER,
    FET_DRIVER,
    DRV_DRIVER,
    HBRIDGE_DRIVER,
};

enum selectFireType_t {
    NO_SELECT_FIRE,
    SWITCH_SELECT_FIRE,
    BUTTON_SELECT_FIRE
};

typedef struct {
    pusherDriverType_t pusherDriverType;
    bool pusherCoastHigh;
    uint8_t nSleep;
    uint8_t flywheel;
    uint8_t pusher1L;
    uint8_t pusher2L;
    uint8_t pusher1H;
    uint8_t pusher2H;
    uint8_t esc1;
    uint8_t esc2;
    uint8_t esc3;
    uint8_t esc4;
    uint8_t telem;
    uint8_t button;
    uint8_t batteryADC;
    uint8_t pusherShunt;
} boards_t;

#endif
