#include "boards_config.h" // board pinouts are in this file. you can check and modify which switch is on which pin there.

// Flywheel Settings
uint32_t revRPM[4] = {50000, 50000, 50000, 50000}; // adjust this to change fps
uint32_t idleRPM[4] = {1000, 1000, 1000, 1000};
uint32_t idleTime_ms = 60000; // how long to idle the flywheels for after releasing the trigger, in milliseconds
uint32_t motorKv = 3200;
uint32_t batteryADC_mv = 14800 / 11; // battery voltage in mv divided by voltage divider ratio (11)
dshot_mode_t dshotMode =  DSHOT150; // DSHOT150 for dshot, or DSHOT_OFF to fall back to servo PWM

// Dettlaff Settings
char wifiSsid[32] = "network name";
char wifiPass[63] = "password";

pins_t pins = pins_v0_5; // select the one that matches your board revision
// Options:
// pins_v0_5
// pins_v0_4_n20
// pins_v0_4_noid
// pins_v0_3_n20
// pins_v0_3_noid
// _noid means use the flywheel output to drive a solenoid pusher
// _n20 for a pusher motor on the pusher output
// pins_v0_2 
// pins_v0_1

// Pusher Settings
pusherType_t pusherType = PUSHER_SOLENOID_OPENLOOP; // either PUSHER_MOTOR_CLOSEDLOOP or PUSHER_SOLENOID_OPENLOOP
uint16_t burstLength = 3;
uint8_t bufferMode = 1;
// 0 = stop firing when trigger is released
// 1 = complete current burst when trigger is released
// 2 = fire as many bursts as trigger pulls
// for full auto, set burstLength high (50+) and bufferMode = 0
uint16_t firingDelay_ms = 200; // delay to allow flywheels to spin up before firing dart
uint16_t solenoidExtendTime_ms = 20;
uint16_t solenoidRetractTime_ms = 35;
bool pusherReverseDirection = false;

// Advanced Settings
uint16_t pusherStallTime_ms = 1000; // for PUSHER_MOTOR_CLOSEDLOOP, how long do you run the motor without seeing an update on the cycle control switch before you decide the motor is stalled?
uint8_t numMotors = 4; // leave at 4 until we have closed loop control
uint16_t spindownSpeed = 1; // higher number makes the flywheels spin down faster after you release the trigger
bool revSwitchNormallyClosed = false; // invert switch signal?
bool triggerSwitchNormallyClosed = false;
bool cycleSwitchNormallyClosed = false;
uint16_t debounceTime_ms = 25;
char AP_SSID[32] = "Dettlaff";
char AP_PW[32] = "KellyIndu";
uint16_t targetLoopTime_us = 1000; // microseconds
uint32_t firingRPM[4] = {revRPM[0]*9/10, revRPM[1]*9/10, // for closed loop flywheel mode only - not implemented yet
                         revRPM[2]*9/10, revRPM[3]*9/10};
float maxDutyCycle_pct = 98;
uint8_t deadtime = 10;
uint16_t pwmFreq_hz = 20000;
