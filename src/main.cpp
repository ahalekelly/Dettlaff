#include "Bounce2.h"
#include "CONFIGURATION.h"
#include "DShotRMT.h"
#include "ESP32Servo.h"
#include "driver.h"
#include "drvDriver.h"
#include "fetDriver.h"
#include "hBridgeDriver.h"
#include "types.h"
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP32AnalogRead.h>
#include <esp_wifi.h>

uint32_t loopStartTimer_us = micros();
uint32_t loopTime_us = targetLoopTime_us;
uint32_t time_ms = millis();
uint32_t lastRevTime_ms = 0; // for calculating idling
uint32_t pusherTimer_ms = 0;
int32_t revRPM[4]; // stores value from revRPMSet on boot for current firing mode
int32_t idleTime_ms; // stores value from idleTimeSet_ms on boot for current firing mode
int32_t targetRPM[4] = { 0, 0, 0, 0 }; // stores current target rpm
int32_t firingRPM[4];
int32_t throttleValue[4] = { 0, 0, 0, 0 }; // scale is 0 - 1999
uint32_t currentSpindownSpeed = 0;
uint16_t burstLength; // stores value from burstLengthSet for current firing mode
burstFireType_t burstMode; // stores value from burstModeSet for current firing mode
int8_t firingMode = 0; // current firing mode
int8_t fpsMode = 0; // copy of firingMode locked at boot
bool fromIdle;
bidirectional_mode_e dshotBidirectional = NO_BIDIRECTION;
int32_t dshotValue = 0;
int16_t shotsToFire = 0;
flywheelState_t flywheelState = STATE_IDLE;
bool firing = false;
bool reverseBraking = false;
bool pusherDwelling = false;
uint32_t batteryADC_mv = 0;
int32_t batteryVoltage_mv = 14800;
int32_t voltageBuffer[voltageAveragingWindow] = { 0 };
int voltageBufferIndex = 0;
int32_t pusherShunt_mv = 0;
int32_t pusherCurrent_ma = 0;
int32_t pusherCurrentSmoothed_ma = 0;
const int32_t maxThrottle = 1999;
int32_t motorRPM[4] = { 0, 0, 0, 0 };
int32_t fullThrottleRpmThreshold[4] = { 0, 0, 0, 0 };
Driver* pusher;
bool wifiState = false;
// String telemBuffer = "";
int8_t telemMotorNum = -1; // 0-3
uint32_t revStartTime_us = 0;
uint32_t triggerTime_ms = 0;
int32_t tempRPM;
ESP32AnalogRead batteryADC;
ESP32AnalogRead pusherShuntADC;
bool currentlyLogging = false;

// closed loop variables
int32_t PIDError[4];
int32_t PIDErrorPrior[4];
int32_t closedLoopRPM[4];
int32_t PIDOutput[4];
int32_t PIDIntegral = 0;

Bounce2::Button revSwitch = Bounce2::Button();
Bounce2::Button triggerSwitch = Bounce2::Button();
Bounce2::Button cycleSwitch = Bounce2::Button();
Bounce2::Button button = Bounce2::Button();
Bounce2::Button select0 = Bounce2::Button();
Bounce2::Button select1 = Bounce2::Button();
Bounce2::Button select2 = Bounce2::Button();

// Declare servo variables for each motor.
Servo servo[4];
DShotRMT dshot[4] = {
    DShotRMT(board.esc1),
    DShotRMT(board.esc2),
    DShotRMT(board.esc3),
    DShotRMT(board.esc4)
};

void WiFiInit();
void updateFiringMode();

void setup()
{
    Serial.begin(460800);
    Serial.println("Booting");

    batteryADC.attach(board.batteryADC);
    batteryADC_mv = batteryADC.readMiliVolts();
    batteryVoltage_mv = voltageCalibrationFactor * batteryADC_mv * 11;
    Serial.print("Battery voltage (before calibration): ");
    Serial.println(batteryVoltage_mv);
    if (voltageCalibrationFactor != 1.0) {
        Serial.print("Battery voltage (after calibration): ");
        Serial.println(voltageCalibrationFactor * batteryADC_mv * 11);
    }

    if (flywheelControl != OPEN_LOOP_CONTROL || printTelemetry) {
        dshotBidirectional = ENABLE_BIDIRECTION;
    }

    switch (board.pusherDriverType) {
    case HBRIDGE_DRIVER:
        pusher = new Hbridge(board.pusher1H, board.pusher1L, board.pusher2H, board.pusher2L, maxDutyCycle_pct, pwmFreq_hz, deadtime);
        pusher->coast();
        break;
    case DRV_DRIVER:
        pusher = new Drv(board.pusher1L, board.pusher2L, pwmFreq_hz, board.pusherCoastHigh);
        break;
    case FET_DRIVER:
        pusher = new Fet(board.pusher1H);
        break;
    default:
        break;
    }

    // Serial2.begin(115200, SERIAL_8N1, board.telem, -1);
    // pinMode(board.telem, INPUT_PULLUP);

    if (revSwitchPin) {
        revSwitch.attach(revSwitchPin, INPUT_PULLUP);
        revSwitch.interval(debounceTime_ms);
        revSwitch.setPressedState(revSwitchNormallyClosed);
    }
    if (triggerSwitchPin) {
        triggerSwitch.attach(triggerSwitchPin, INPUT_PULLUP);
        triggerSwitch.interval(debounceTime_ms);
        triggerSwitch.setPressedState(triggerSwitchNormallyClosed);
    }
    if (cycleSwitchPin) {
        cycleSwitch.attach(cycleSwitchPin, INPUT_PULLUP);
        cycleSwitch.interval(pusherDebounceTime_ms);
        cycleSwitch.setPressedState(cycleSwitchNormallyClosed);
    }
    if (selectFireType != NO_SELECT_FIRE) {
        if (select0Pin) {
            select0.attach(select0Pin, INPUT_PULLUP);
            select0.interval(debounceTime_ms);
            select0.setPressedState(false);
        }
        if (select1Pin) {
            select1.attach(select1Pin, INPUT_PULLUP);
            select1.interval(debounceTime_ms);
            select1.setPressedState(false);
        }
        if (select2Pin) {
            select2.attach(select2Pin, INPUT_PULLUP);
            select2.interval(debounceTime_ms);
            select2.setPressedState(false);
        }
    }

    if (dshotMode == DSHOT_OFF) {
        for (int i = 0; i < 4; i++) {
            ESP32PWM::allocateTimer(i);
            servo[i].setPeriodHertz(servoFreq_hz);
        }
        servo[0].attach(board.esc1);
        servo[1].attach(board.esc2);
        servo[2].attach(board.esc3);
        servo[3].attach(board.esc4);
    } else {
        for (int i = 0; i < 4; i++) {
            if (motors[i]) {
                dshot[i].begin(dshotMode, dshotBidirectional, 14); // bitrate, bidirectional, and motor pole count
            }
        }
    }

    // change FPS using select fire switch position at boot time
    if (variableFPS) {
        updateFiringMode();
    }

    fpsMode = firingMode;
    Serial.print("fpsMode: ");
    Serial.println(fpsMode);
    for (int i = 0; i < 4; i++) {
        if (motors[i]) {
            revRPM[i] = revRPMset[fpsMode][i];
            firingRPM[i] = max(revRPM[i] - firingRPMTolerance, minFiringRPM);
            fullThrottleRpmThreshold[i] = revRPM[i] - fullThrottleRpmTolerance;
        }
    }
    idleTime_ms = idleTimeSet_ms[fpsMode];

    if (board.flywheel) {
        pinMode(board.flywheel, OUTPUT);
        if (!brushedFlywheels) {
            digitalWrite(board.flywheel, HIGH);
        }
    }

    for (int count = 0; count < 10000; count++) {
        for (int i = 0; i < 4; i++) {
            if (motors[i]) {
                dshot[i].send_dshot_value(0, NO_TELEMETRIC);
            }
        }
    }

    if (board.nSleep) {
        pinMode(board.nSleep, OUTPUT);
        digitalWrite(board.nSleep, HIGH);
        delayMicroseconds(1000);
        digitalWrite(board.nSleep, LOW);
        delayMicroseconds(30);
        digitalWrite(board.nSleep, HIGH);
    }

    batteryADC.attach(board.batteryADC);
    pusherShuntADC.attach(board.pusherShunt);

    if (wifiDuration_ms > 0) {
        WiFiInit();
    }
}

void loop()
{
    loopStartTimer_us = micros();
    time_ms = millis();
    if (revSwitchPin) {
        revSwitch.update();
    }
    if (triggerSwitchPin) {
        triggerSwitch.update();
    }
    updateFiringMode();
    // changes burst options
    burstLength = burstLengthSet[firingMode];
    burstMode = burstModeSet[firingMode];

    // Transfer data from telemetry serial port to telemetry serial buffer:
    // while (Serial2.available()) {
    //     telemBuffer += Serial2.read(); // this doesn't seem to work - do we need 1k pullup resistor? also is this the most efficient way to do this?
    // }
    // Then parse serial buffer, if serial buffer contains complete packet then update motorRPM value, clear serial buffer, and increment telemMotorNum to get the data for the next motor
    // will we be able to detect the gaps between packets to know when a packet is complete? Need to test and see
    //    Serial.println(telemBuffer);

    if (triggerSwitch.pressed() || (burstMode == BINARY && triggerSwitch.released() && time_ms < triggerTime_ms + binaryTriggerTimeout_ms)) { // pressed and released are transitions, isPressed is for state
        Serial.print(time_ms);
        if (triggerSwitch.pressed()) {
            Serial.print(" trigger pressed, burstMode ");
        } else if (burstMode == BINARY && triggerSwitch.released() && time_ms < triggerTime_ms + binaryTriggerTimeout_ms) {
            Serial.print(" binary trigger released, burstMode ");
        }
        triggerTime_ms = time_ms;
        Serial.print(burstMode);
        Serial.print(" shotsToFire before ");
        Serial.print(shotsToFire);
        if (burstMode == AUTO) {
            shotsToFire = burstLength;
        } else {
            if (shotsToFire < burstLength || shotsToFire == 1) {
                shotsToFire += burstLength;
            }
        }
        Serial.print(" after ");
        Serial.println(shotsToFire);
    } else if (triggerSwitch.released()) {
        if (burstMode == AUTO && shotsToFire > 1) {
            shotsToFire = 1;
        }
    }

    switch (flywheelState) {

    case STATE_IDLE:
        if (batteryVoltage_mv < lowVoltageCutoff_mv && throttleValue[0] == 0 && time_ms > 2000) {
            digitalWrite(board.flywheel, LOW); // cut power to ESCs and pusher
            Serial.print("Battery low, shutting down! ");
            Serial.print(batteryVoltage_mv);
            Serial.println("mv");
            esp_deep_sleep_start(); // go to sleep and never wake up
        }

        if (shotsToFire > 0 || revSwitch.isPressed()) {
            revStartTime_us = loopStartTimer_us;
            memcpy(targetRPM, revRPM, sizeof(targetRPM)); // Copy revRPM to targetRPM
            lastRevTime_ms = time_ms;
            flywheelState = STATE_ACCELERATING;
            currentSpindownSpeed = 0; // reset spindownSpeed
        } else if (time_ms < lastRevTime_ms + idleTime_ms && lastRevTime_ms > 0) { // idle flywheels
            if (currentSpindownSpeed < spindownSpeed) {
                currentSpindownSpeed += 1;
            }
            for (int i = 0; i < 4; i++) {
                if (motors[i]) {
                    targetRPM[i] = max(targetRPM[i] - static_cast<int32_t>((currentSpindownSpeed * loopTime_us) / 1000), idleRPM[i]);
                }
            }
        } else { // stop flywheels
            if (currentSpindownSpeed < spindownSpeed) {
                currentSpindownSpeed += 1;
            }
            for (int i = 0; i < 4; i++) {
                if (motors[i] && targetRPM[i] != 0) {
                    targetRPM[i] = max(targetRPM[i] - static_cast<int32_t>((currentSpindownSpeed * loopTime_us) / 1000), 0);
                }
            }
            PIDIntegral = 0;
            fromIdle = false;
        }
        break;

    case STATE_ACCELERATING:
        // clang-format off
        if (flywheelControl != OPEN_LOOP_CONTROL && time_ms > lastRevTime_ms + (fromIdle ? minFiringDelayIdleSet_ms[fpsMode] : minFiringDelaySet_ms[fpsMode])) {
            // If all motors are at target RPM, update the blaster's state to FULLSPEED.
            if ((!motors[0] || motorRPM[0] > firingRPM[0]) &&
                (!motors[1] || motorRPM[1] > firingRPM[1]) &&
                (!motors[2] || motorRPM[2] > firingRPM[2]) &&
                (!motors[3] || motorRPM[3] > firingRPM[3])
            ) {
                flywheelState = STATE_FULLSPEED;
                fromIdle =  true;
                Serial.println("STATE_FULLSPEED transition 1");
            } else if (loopStartTimer_us - revStartTime_us > 2000000) {
                flywheelState = STATE_IDLE;
                shotsToFire = 0;
                Serial.println("Error! Flywheels failed to reach target speed!");
            }
        }
        if ((flywheelControl == OPEN_LOOP_CONTROL
            || (timeOverrideWhenIdling &&
                fromIdle &&
                (!motors[0] || motorRPM[0] > 100) &&
                (!motors[1] || motorRPM[1] > 100) &&
                (!motors[2] || motorRPM[2] > 100) &&
                (!motors[3] || motorRPM[3] > 100)
                )
            )
            && time_ms > lastRevTime_ms + (fromIdle ? firingDelayIdleSet_ms[fpsMode] : firingDelaySet_ms[fpsMode])) {
                flywheelState = STATE_FULLSPEED;
                fromIdle = true;
                Serial.print(time_ms);
                Serial.println(" STATE_FULLSPEED transition, firingDelay time elapsed");
        }
        break;
        // clang-format on

    case STATE_FULLSPEED:
        if (!revSwitch.isPressed() && shotsToFire == 0 && !firing) {
            flywheelState = STATE_IDLE;
            Serial.println("state transition: FULLSPEED to IDLE 1");
        } else if (shotsToFire > 0 || firing) {
            lastRevTime_ms = time_ms;
            switch (pusherType) {

            case PUSHER_MOTOR_CLOSEDLOOP:
                cycleSwitch.update();
                if (shotsToFire > 0 && !firing) { // start pusher stroke
                    pusher->drive(100.0 * pusherVoltage_mv / batteryVoltage_mv, pusherReverseDirection); // drive function clamps the input so it's ok if it's over 100
                    firing = true;
                    pusherTimer_ms = time_ms;
                    Serial.print(time_ms);
                    Serial.println(" pusher stroke starting");
                } else if (firing && cycleSwitch.pressed()) { // when the pusher reaches rear position
                    shotsToFire = max(0, shotsToFire - 1);
                    Serial.print(time_ms);
                    Serial.print(" pusher reached rear, shotsToFire reduced by 1, now ");
                    Serial.println(shotsToFire);
                    pusherTimer_ms = time_ms;
                    if (shotsToFire <= 0) { // brake pusher
                        if (pusherReversePolarityDuration_ms > 0) {
                            pusher->drive(100.0 * pusherReverseBrakingVoltage_mv / batteryVoltage_mv, !pusherReverseDirection); // drive motor backwards to stop faster
                            reverseBraking = true;
                            Serial.print(time_ms);
                            Serial.println(" reverse braking started");
                            //                  firing = false; this doesn't work because this pusher control routine only runs when the flywheels are running, so this causes reverse braking to never end. refactor later?
                        } else {
                            pusher->brake();
                            firing = false;
                            flywheelState = STATE_IDLE; // check later
                            Serial.println("state transition: FULLSPEED to IDLE 2");
                        }
                    } else if (pusherDwellTime_ms > 0) {
                        if (pusherBrakeOnDwell) {
                            pusher->brake();
                        } else {
                            pusher->coast();
                        }
                        pusherDwelling = true;
                    }
                } else if (pusherDwelling) { // dwelling to reduce rate of fire
                    if (time_ms - pusherTimer_ms > pusherDwellTime_ms) { // if we've coasted the pusher for long enough
                        pusher->drive(100.0 * pusherVoltage_mv / batteryVoltage_mv, pusherReverseDirection); // resume pushing
                        pusherDwelling = false;
                    }
                } else if (reverseBraking) { // if we're currently doing reverse braking
                    if (shotsToFire >= 1) { // stop reverse braking and resume firing
                        pusher->drive(100.0 * pusherVoltage_mv / batteryVoltage_mv, pusherReverseDirection); // drive function clamps the input so it's ok if it's over 100
                        firing = true;
                        pusherTimer_ms = time_ms;
                        reverseBraking = false;
                        Serial.print(time_ms);
                        Serial.println(" ending reverse braking, resuming firing");
                    } else if (cycleSwitch.released() && pusherEndReverseBrakingEarly) {
                        Serial.println("Cycle switch released during reverse braking");
                        pusher->brake();
                        reverseBraking = false;
                        firing = false;
                        flywheelState = STATE_IDLE; // check later
                        Serial.println("state transition: FULLSPEED to IDLE 3");
                    } else if (cycleSwitch.pressed()) {
                        Serial.println("Cycle switch pressed during reverse braking");
                        pusher->brake();
                        reverseBraking = false;
                        firing = false;
                        flywheelState = STATE_IDLE; // check later
                        Serial.println("state transition: FULLSPEED to IDLE 4");
                    } else if (time_ms > pusherTimer_ms + pusherReversePolarityDuration_ms) {
                        Serial.print(time_ms);
                        Serial.println(" pusherReverse end of duration");
                        pusher->brake();
                        reverseBraking = false;
                        firing = false;
                        flywheelState = STATE_IDLE; // check later
                        Serial.print(time_ms);
                        Serial.println(" state transition: FULLSPEED to IDLE 5");
                    }
                } else if (!firing && cycleSwitch.released() && pusherReverseOnOverrun) {
                    Serial.println("pusherReverseOnOverrun");
                    pusher->drive(100, !pusherReverseDirection); // drive motor backwards to stop faster
                    reverseBraking = true;
                } else if (firing && time_ms > pusherTimer_ms + pusherStallTime_ms) { // stall protection
                    pusher->coast();
                    shotsToFire = 0;
                    firing = false;
                    flywheelState = STATE_IDLE; // check later
                    Serial.println("Pusher motor stalled!");
                }
                break;

            case PUSHER_SOLENOID_OPENLOOP:
                if (shotsToFire > 0 && !firing && time_ms > pusherTimer_ms + solenoidRetractTime_ms) { // extend solenoid
                    pusher->drive(100, pusherReverseDirection);
                    firing = true;
                    shotsToFire = max(0, shotsToFire - 1);
                    pusherTimer_ms = time_ms;
                    Serial.println("solenoid extending");
                } else if (firing && time_ms > pusherTimer_ms + solenoidExtendTime_ms) { // retract solenoid
                    pusher->coast();
                    firing = false;
                    pusherTimer_ms = time_ms;
                    Serial.println("solenoid retracting");
                }
                break;
            case NO_PUSHER:
                break;
            }
        }
        break;
    }
    batteryADC_mv = batteryADC.readMiliVolts();
    if (voltageAveragingWindow == 1) {
        batteryVoltage_mv = voltageCalibrationFactor * batteryADC_mv * 11;
    } else {
        voltageBuffer[voltageBufferIndex] = voltageCalibrationFactor * batteryADC_mv * 11;
        voltageBufferIndex = (voltageBufferIndex + 1) % voltageAveragingWindow;
        batteryVoltage_mv = 0;
        for (int i = 0; i < voltageAveragingWindow; i++) {
            batteryVoltage_mv += voltageBuffer[i];
        }
        batteryVoltage_mv /= voltageAveragingWindow; // apply exponential moving average to smooth out noise. Time constant ≈ 1.44 ms
    }
    if (wifiState == false) {
        pusherShunt_mv = pusherShuntADC.readMiliVolts();
        pusherCurrent_ma = pusherShunt_mv * 3070 / 1000 - 392; // 3070 current reduction factor for IPROPI, 1k shunt resistor, and the datasheet says the zero offset should be 15mA but experimentally it's 392mA
        pusherCurrentSmoothed_ma = (pusherCurrent_ma * (100 - pusherCurrentSmoothingFactor) + pusherCurrentSmoothed_ma * pusherCurrentSmoothingFactor) / 100;
    }

    switch (flywheelControl) {
    case PID_CONTROL:
        for (int i = 0; i < 4; i++) {
            if (motors[i]) {
                PIDError[i] = targetRPM[i] - motorRPM[i];

                PIDOutput[i] = KP * PIDError[i] + KI * (PIDIntegral + PIDError[i] * loopTime_us / 1000000) + KD * (PIDError[i] - PIDErrorPrior[i]) / loopTime_us * 1000000;
                closedLoopRPM[i] = PIDOutput[i] + motorRPM[i];

                if (throttleValue[i] == 0) {
                    throttleValue[i] = min(maxThrottle, maxThrottle * closedLoopRPM[i] / batteryVoltage_mv * 1000 / motorKv);
                } else {
                    throttleValue[i] = max(min(maxThrottle, maxThrottle * closedLoopRPM[i] / batteryVoltage_mv * 1000 / motorKv),
                        throttleValue[i] - 1);
                }

                PIDErrorPrior[i] = PIDError[i];
                PIDIntegral += PIDIntegral + PIDError[i] * loopTime_us / 1000000;
            }
        }
        break;

    case TWO_LEVEL_CONTROL:
        for (int i = 0; i < 4; i++) {
            if (motors[i]) {
                if (targetRPM[i] == revRPM[i] && motorRPM[i] < fullThrottleRpmThreshold[i]) {
                    throttleValue[i] = maxThrottle;
                } else {
                    throttleValue[i] = max(min(maxThrottle, maxThrottle * targetRPM[i] / batteryVoltage_mv * 1000 / motorKv), 0);
                }
            }
        }
        break;

    case OPEN_LOOP_CONTROL:
        for (int i = 0; i < 4; i++) {
            if (motors[i]) {
                throttleValue[i] = max(min(maxThrottle, maxThrottle * targetRPM[i] / batteryVoltage_mv * 1000 / motorKv), 0);
            }
        }
        break;
    }

    // send signal to ESCs
    if (brushedFlywheels) {
        if (targetRPM[0] > 0) {
            digitalWrite(board.flywheel, HIGH);
        } else {
            digitalWrite(board.flywheel, LOW);
        }
    } else if (dshotMode == DSHOT_OFF) {
        for (int i = 0; i < 4; i++) {
            if (motors[i]) {
                servo[i].writeMicroseconds(throttleValue[i] / 2 + 1000);
                /*
                Serial.print(targetRPM[i]);
                Serial.print(" ");
                Serial.print(throttleValue[i] / 2 + 1000);
                Serial.print(" ");
                */
            }
        }
        //    Serial.println("");
    } else {
        for (int i = 0; i < 4; i++) {
            if (motors[i]) {
                if (dshotBidirectional == ENABLE_BIDIRECTION) {
                    tempRPM = static_cast<int32_t>(dshot[i].get_dshot_RPM());
                    if (tempRPM > 0) { // todo: rate of change filtering
                        motorRPM[i] = tempRPM;
                    }
                }
                if (throttleValue[i] == 0) {
                    dshotValue = 0;
                } else {
                    dshotValue = throttleValue[i] + 48;
                }
                if (i == telemMotorNum) {
                    dshot[i].send_dshot_value(dshotValue, ENABLE_TELEMETRIC);
                } else {
                    dshot[i].send_dshot_value(dshotValue, NO_TELEMETRIC);
                }
            }
        }

        // Telemetry logging for use with dyno python script
        if (printTelemetry && telemetryInterval_ms > 0 && time_ms % telemetryInterval_ms == 0 && dshotBidirectional == ENABLE_BIDIRECTION && lastRevTime_ms != 0) {
            if (time_ms - lastRevTime_ms < 100 || time_ms - triggerTime_ms < 250) {
                Serial.print((loopStartTimer_us - revStartTime_us) / 1000); // time since trigger pull
                // Serial.print(loopStartTimer_us / 1000); // time since boot
                Serial.print(",");
                Serial.print(batteryADC_mv * 11);
                Serial.print(",");
                Serial.print(pusherCurrent_ma);
                for (int i = 0; i < 4; i++) {
                    if (motors[i]) {
                        Serial.print(",");
                        Serial.print(throttleValue[i]);
                        Serial.print(",");
                        Serial.print(motorRPM[i]);
                    }
                }
                Serial.println();
                currentlyLogging = true;
            } else if (currentlyLogging) { // was logging but logging period over
                currentlyLogging = false;
                Serial.println("end of telemetry");
            }
        }
    }
    if (wifiState == true) {
        if (time_ms > wifiDuration_ms || flywheelState != STATE_IDLE) {
            wifiState = false;
            Serial.println("Wifi turning off");
            ArduinoOTA.end();
            WiFi.disconnect(true); // Disconnect from the network
            WiFi.mode(WIFI_OFF); // Switch WiFi off
        } else {
            ArduinoOTA.handle();
        }
    }
    loopTime_us = micros() - loopStartTimer_us; // 'us' is microseconds
    if (loopTime_us > targetLoopTime_us) {
        Serial.print("loop over time, ");
        Serial.println(loopTime_us);
    } else {
        delayMicroseconds(max((long)(0), (long)(targetLoopTime_us - loopTime_us)));
    }
}

void updateFiringMode()
{
    if (selectFireType == NO_SELECT_FIRE) {
        return;
    }
    if (select0Pin) {
        select0.update();
        if (select0.isPressed()) {
            firingMode = 0;
            return;
        }
    }
    if (select1Pin) {
        select1.update();
        if (select1.isPressed()) {
            firingMode = 1;
            return;
        }
    }
    if (select2Pin) {
        select2.update();
        if (select2.isPressed()) {
            firingMode = 2;
            return;
        }
    }
    if (selectFireType == SWITCH_SELECT_FIRE) { // if BUTTON_SELECT_FIRE then don't change modes
        firingMode = defaultFiringMode;
        return;
    }
}

void WiFiInit()
{
    wifiState = true;
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSsid, wifiPass);
    if (wifiSsid[0] == '\0' || WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.print("Creating WiFi hotspot, password is ");
        Serial.println(AP_PW);
        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PW);
        ArduinoOTA.setHostname("Dettlaff");
    } else {
        Serial.print("WiFi Connected ");
        Serial.println(wifiSsid);
        ArduinoOTA.setHostname("Dettlaff");
        /*
            if(!MDNS.begin("dettlaff")) {
              Serial.println("Error starting mDNS");
              return;
            }
        */
        Serial.println(WiFi.localIP());

        // No authentication by default
        // ArduinoOTA.setPassword("admin");
    }

    ArduinoOTA
        .onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH) {
                type = "sketch";
            } else { // U_SPIFFS
                type = "filesystem";
            }
            // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
            Serial.println("Start updating " + type);
        })
        .onEnd([]() {
            Serial.println("\nEnd");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR)
                Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR)
                Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR)
                Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR)
                Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR)
                Serial.println("End Failed");
        });

    ArduinoOTA.begin();

    Serial.println("WiFi Ready");
}
