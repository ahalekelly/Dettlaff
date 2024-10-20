#include "DShotRMT.h" // We need this for the dshot modes
#include "boards_config.h" // board pinouts are in this file

// Flywheel Settings
// If variableFPS is true, the following settings are set on boot and locked. Otherwise, it always uses the first mode
bool variableFPS = true;
int32_t revRPMset[3][4] = { { 40000, 40000, 40000, 40000 }, { 25000, 25000, 25000, 25000 }, { 14000, 14000, 14000, 14000 } }; // adjust this to change fps, groups are firingMode 1, 2, 3, and the 4 elements in each group are individual motor RPM
uint32_t idleTimeSet_ms[3] = { 30000, 5000, 2000 }; // how long to idle the flywheels for after releasing the trigger, in milliseconds
uint32_t firingDelaySet_ms[3] = { 150, 125, 100 }; // delay to allow flywheels to spin up before firing dart
uint32_t firingDelayIdleSet_ms[3] = { 125, 100, 80 }; // delay to allow flywheels to spin up before firing dart when starting from idle state
uint32_t spindownSpeed = 30; // RPM per ms

int32_t motorKv = 3200; // critical for closed loop
int32_t idleRPM[4] = { 500, 500, 500, 500 }; // rpm for flywheel idling, set this as low as possible where the wheels still spin reliably
dshot_mode_t dshotMode = DSHOT300; // Options are DSHOT150, DSHOT300, DSHOT600, or DSHOT_OFF. DSHOT300 is recommended, DSHOT150 does not work with either AM32 ESCs or closed loop control, and DSHOT600 seems less reliable. DSHOT_OFF falls back to servo PWM. PWM is not working, probably a ESP32 timer resource conflict with the pusher PWM circuit
bool brushedFlywheels = false; // solder a brushed motor flywheel cage to the ESC+ and Brushed Motor - pads

// Closed Loop Settings
flywheelControlType_t flywheelControl = OPEN_LOOP_CONTROL; // OPEN_LOOP_CONTROL or TWO_LEVEL_CONTROL
const bool motors[4] = {true, true, true, true}; // which motors are hooked up
bool timeOverrideWhenIdling = true; // while idling, fire the pusher after firingDelay_ms even before the flywheels are up to speed
int32_t fullThrottleRpmTolerance = 5000; // if rpm is more than this amount below target rpm, send full throttle. too high and rpm will undershoot, too low and it will overshoot
int32_t firingRPMTolerance = 10000; // fire pusher when all flywheels are within this amount of target rpm. higher values will mean less pusher delay but potentially fire too early
int32_t minFiringRPM = 10000; // overrides firingRPMTolerance for low rpm settings
int32_t minFiringDelaySet_ms[3] = {0, 0, 0}; // when not idling, don't fire the pusher before this amount of time, even if wheels are up to speed. makes the delay more consistent
int32_t minFiringDelayIdleSet_ms[3] = {0, 0, 0}; // same but when idling

// Select Fire Settings
uint32_t burstLengthSet[3] = { 100, 5, 1 };
burstFireType_t burstModeSet[3] = { AUTO, AUTO, BURST };
// burstMode AUTO = stops firing when trigger is released
// burstMode BURST = always completes the burst
// burstMode BINARY = fires one burst when you pull the trigger and another when you release the trigger
// for full auto, set burstLength high (50+) and burstMode to AUTO
// for semi auto, set burstLength to 1 and burstMode to BURST
// for burst fire, set burstLength and burstMode to BURST
// i find a very useful mode is full auto with a 5 dart limit (burstMode AUTO, burstLength 5)

uint32_t binaryTriggerTimeout_ms = 2000; // if you hold the trigger for more than this amount of time, releasing the trigger will not fire a burst


selectFireType_t selectFireType = SWITCH_SELECT_FIRE; // pick NO_SELECT_FIRE, SWITCH_SELECT_FIRE, or BUTTON_SELECT_FIRE
uint8_t defaultFiringMode = 1; // only for SWITCH_SELECT_FIRE, what mode to select if no pins are connected

// Dettlaff Settings
bool printTelemetry = false; // output telemetry over USB serial port for tuning. Enabling this turns on bidirectional dshot
uint32_t lowVoltageCutoff_mv = 2500 * 4; // default is 2.5V per cell * 4 cells because the ESP32 voltage measurement is not very accurate
// to protect your batteries, i reccomend doing the calibration below and then setting the cutoff to 3.2V to 3.4V per cell
float voltageCalibrationFactor = 1.0; // measure the battery voltage with a multimeter and divide that by the "Battery voltage before calibration" printed in the Serial Monitor, then put the result here

// Input Pins, set to 0 if not using
uint8_t triggerSwitchPin = 32; // main trigger pin
uint8_t revSwitchPin = 15; // optional rev trigger
uint8_t cycleSwitchPin = 23; // pusher motor home switch
uint8_t select0Pin = 25; // optional for select fire
uint8_t select1Pin = 0; // optional for select fire
uint8_t select2Pin = 33; // optional for select fire

boards_t board = board_v0_9; // select the one that matches your board revision
// Options:
// board_v0_11
// board_v0_10
// board_v0_9
// board_v0_8
// board_v0_7
// board_v0_6
// board_v0_5
// board_v0_4_n20
// board_v0_4_noid
// board_v0_3_n20
// board_v0_3_noid
// _noid means use the flywheel output to drive a solenoid pusher
// _n20 for a pusher motor on the pusher output
// board_v0_2
// board_v0_1

// Pusher Settings
pusherType_t pusherType = PUSHER_MOTOR_CLOSEDLOOP; // either PUSHER_MOTOR_CLOSEDLOOP or PUSHER_SOLENOID_OPENLOOP
uint32_t pusherVoltage_mv = 16000; // if battery voltage is above this voltage, then use PWM to reduce the voltage that the pusher sees
bool pusherReverseDirection = false; // make motor spin backwards? v0.5 & v0.6 (hBridgeDriver) need this to be false or the pusher logic is inverted? and the v0.2 - v0.4 at8870 pusher seems to need this to be true for reverse polarity braking to work?

// Solenoid Settings
uint16_t solenoidExtendTime_ms = 20;
uint16_t solenoidRetractTime_ms = 35;

//Pusher Motor Settings
uint32_t pusherReverseBrakingVoltage_mv = 16000;
uint8_t pusherReversePolarityDuration_ms = 5; // try increasing this if your pusher doesn't stop at the right position because your pusher motor takes too long to stop. 10ms was good for my FDL with cheap pusher motor
uint32_t pusherDwellTime_ms = 0; // dwell for this long at the end of each pusher cycle in full auto / burst mode to slow down rate of fire and allow mag to push the next dart. doesn't dwell if it's the last shot
bool pusherBrakeOnDwell = false; // if true then the pusher brakes during its dwell time, if false it coasts

// Wi-Fi Settings
uint32_t wifiDuration_ms = 10 * 60 * 1000; // how long before wifi turns off to save power. default is 10 min. set to 0 to disable wifi
char wifiSsid[32] = ""; // Name and password of your home wifi network. If this is left empty, Dettlaff will always create its own wifi network that you can connect to and upload code. Some people have experienced issues when Dettlaff is configured to connect to their home wifi
char wifiPass[63] = "";
char AP_SSID[32] = "Dettlaff"; // name and password for the wifi network created by Dettlaff
char AP_PW[32] = "Kellytime";

// Advanced Settings
uint16_t pusherStallTime_ms = 750; // for PUSHER_MOTOR_CLOSEDLOOP, how long do you run the motor without seeing an update on the cycle control switch before you decide the motor is stalled?
bool revSwitchNormallyClosed = false; // invert switch signal?
bool triggerSwitchNormallyClosed = false;
bool cycleSwitchNormallyClosed = false;
uint16_t debounceTime_ms = 100; // decrease if you're unable to make fast double taps in semi auto, increase if you're getting accidental double taps in semi auto
uint16_t pusherDebounceTime_ms = 25;
const int voltageAveragingWindow = 1;
uint32_t pusherCurrentSmoothingFactor = 90;
uint8_t telemetryInterval_ms = 5;
uint16_t targetLoopTime_us = 1000; // microseconds
float maxDutyCycle_pct = 98;
uint8_t deadtime = 10;
uint16_t pwmFreq_hz = 20000;
uint16_t servoFreq_hz = 200;
bool pusherReverseOnOverrun = false; // don't use this, broken
bool pusherEndReverseBrakingEarly = false; // don't use this, broken

// PID Settings (PID not working)
float KP = 1.5;
float KI = 0.0;
float KD = 0.5;
