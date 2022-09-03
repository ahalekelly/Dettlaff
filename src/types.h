#include <Arduino.h>

enum flywheelState_t {
  STATE_IDLE,
  STATE_ACCELERATING, // ACCELERATING = wheels not yet at full speed
  STATE_FULLSPEED, // REV = wheels at full speed
};

typedef struct {
  int8_t revSwitch;
  int8_t triggerSwitch;
  int8_t cycleSwitch;
  int8_t flywheel;
  int8_t pusher;
  int8_t pusherBrake;
  int8_t esc1;
  int8_t esc2;
  int8_t esc3;
  int8_t esc4;
  int8_t telem;
  int8_t button;
  int8_t batteryADC;
} pins_t;

enum pusherType_t {
  NO_PUSHER,
  PUSHER_MOTOR_CLOSEDLOOP,
  PUSHER_SOLENOID_OPENLOOP
};
