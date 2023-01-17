#include <Arduino.h>
#include <ArduinoOTA.h>
#define BOUNCE_LOCK_OUT  // improves trigger responsiveness at the risk of
                         // spurious signals from noise
#include "Bounce2.h"
#include "DShotRMT.h"
#include "ESP32Servo.h"
#include "boards_config.cpp"
#include "types.h"

// Configuration Variables

char wifiSsid[32] = "ssid";
char wifiPass[63] = "pass";
uint8_t numMotors = 2;  // 2 for single-stage, 4 for dual-stage
uint32_t revRPM[4] = {
    50000, 50000, 50000,
    50000};  // adjust this to change fps - note that these numbers currently
             // assume you have a 4S battery! will fix soon
uint32_t idleRPM[4] = {1000, 1000, 1000, 1000};
uint32_t idleTime_ms = 30000;  // how long to idle the flywheels for after
                               // releasing the trigger, in milliseconds
uint32_t motorKv = 2550;
pins_t pins = pins_v0_4_noid;  // select the one that matches your board
                               // revision and pusher type
// Options:
// _noid means use the flywheel output to drive a solenoid pusher
// _n20 for a pusher motor on the pusher output
// pins_v0_4_n20
// pins_v0_4_noid
// pins_v0_3_n20
// pins_v0_3_noid
// pins_v0_2
// pins_v0_1
pusherType_t pusherType = PUSHER_SOLENOID_OPENLOOP;
// PUSHER_MOTOR_CLOSEDLOOP or PUSHER_SOLENOID_OPENLOOP
uint16_t burstLength = 3;
uint8_t bufferMode = 1;
// 0 = stop firing when trigger is released
// 1 = complete current burst when trigger is released
// 2 = fire as many bursts as trigger pulls
// for full auto, set burstLength high (50+) and bufferMode = 0
uint16_t firingDelay_ms =
    200;  // delay to allow flywheels to spin up before pushing dart
uint16_t solenoidExtendTime_ms = 22;
uint16_t solenoidRetractTime_ms = 78;

// Advanced Configuration Variables

uint16_t pusherStallTime_ms =
    500;  // for PUSHER_MOTOR_CLOSEDLOOP, how long do you run the motor without
          // seeing an update on the cycle control switch before you decide the
          // motor is stalled?
uint16_t spindownSpeed = 1;  // higher number makes the flywheels spin down
                             // faster when releasing the rev trigger
bool revSwitchNormallyClosed = false;  // should we invert rev signal?
bool triggerSwitchNormallyClosed = false;
bool cycleSwitchNormallyClosed = false;
uint16_t debounceTime = 25;  // ms
char AP_SSID[32] = "Dettlaff";
char AP_PW[32] = "KellyIndu";
dshot_mode_t dshotMode = DSHOT300;  // DSHOT_OFF to fall back to servo PWM
uint16_t targetLoopTime_us = 1000;  // microseconds
uint32_t firingRPM[4] = {
    revRPM[0] * 9 / 10,
    revRPM[1] * 9 /
        10,  // for closed loop flywheel mode only - not implemented yet
    revRPM[2] * 9 / 10, revRPM[3] * 9 / 10};

// End Configuration Variables

uint32_t loopStartTimer_us = micros();
uint16_t loopTime_us = targetLoopTime_us;
uint32_t time_ms = millis();
uint32_t lastRevTime_ms = 0;  // for calculating idling
uint32_t pusherTimer_ms = 0;
uint32_t zeroRPM[4] = {0, 0, 0, 0};
uint32_t (*targetRPM)[4];  // a pointer to a uint32_t[4] array. always points to
                           // either revRPM, idleRPM, or zeroRPM
uint32_t throttleValue[4] = {0, 0, 0, 0};  // scale is 0 - 1999
uint32_t batteryADC_mv = 1340;  // voltage at the ADC, after the voltage divider
uint16_t shotsToFire = 0;
flywheelState_t flywheelState = STATE_IDLE;
bool firing = false;
bool closedLoopFlywheels = false;
uint32_t scaledMotorKv =
    motorKv * 11;  // motor kv * battery voltage resistor divider ratio
const uint32_t maxThrottle = 1999;
uint32_t motorRPM[4] = {0, 0, 0, 0};

Bounce2::Button revSwitch = Bounce2::Button();
Bounce2::Button triggerSwitch = Bounce2::Button();
Bounce2::Button cycleSwitch = Bounce2::Button();
Bounce2::Button button = Bounce2::Button();

// Declare servo variables for each motor.
Servo servo[4];
DShotRMT dshot[4] = {
    DShotRMT(pins.esc1, RMT_CHANNEL_1), DShotRMT(pins.esc2, RMT_CHANNEL_2),
    DShotRMT(pins.esc3, RMT_CHANNEL_3), DShotRMT(pins.esc4, RMT_CHANNEL_4)};

void WiFiInit();

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  if (pins.flywheel) {
    pinMode(pins.flywheel, OUTPUT);
    digitalWrite(pins.flywheel, HIGH);
  }
  WiFiInit();
  if (pins.revSwitch) {
    revSwitch.attach(pins.revSwitch, INPUT_PULLUP);
    revSwitch.interval(debounceTime);
    revSwitch.setPressedState(revSwitchNormallyClosed);
  }
  if (pins.triggerSwitch) {
    triggerSwitch.attach(pins.triggerSwitch, INPUT_PULLUP);
    triggerSwitch.interval(debounceTime);
    triggerSwitch.setPressedState(triggerSwitchNormallyClosed);
  }
  if (pins.cycleSwitch) {
    cycleSwitch.attach(pins.cycleSwitch, INPUT_PULLUP);
    cycleSwitch.interval(debounceTime);
    cycleSwitch.setPressedState(cycleSwitchNormallyClosed);
  }
  if (pins.pusher) {
    pinMode(pins.pusher, OUTPUT);
    digitalWrite(pins.pusher, LOW);
    pinMode(pins.pusherBrake, OUTPUT);
    digitalWrite(pins.pusherBrake, LOW);
  }

  if (dshotMode == DSHOT_OFF) {
    for (int i = 0; i < 4; i++) {
      ESP32PWM::allocateTimer(i);
      servo[i].setPeriodHertz(200);
    }
    servo[1].attach(pins.esc1);
    servo[2].attach(pins.esc2);
    servo[3].attach(pins.esc3);
    servo[4].attach(pins.esc4);
  } else {
    for (int i = 0; i < numMotors; i++) {
      dshot[i].begin(dshotMode, false);  // bitrate & bidirectional
    }
  }
}

void loop() {
  loopStartTimer_us = micros();
  time_ms = millis();
  if (pins.revSwitch) {
    revSwitch.update();
  }
  if (pins.triggerSwitch) {
    triggerSwitch.update();
  }

  // *Need to implement*
  // Get flywheel RPM data, store it in motorRPM

  if (triggerSwitch.pressed()) {  // pressed and released are transitions,
                                  // isPressed is for state
    if (bufferMode == 0) {
      shotsToFire = burstLength;
    } else if (bufferMode == 1) {
      if (shotsToFire < burstLength) {
        shotsToFire += burstLength;
      }
    } else if (bufferMode == 2) {
      shotsToFire += burstLength;
    }
  } else if (triggerSwitch.released()) {
    if (bufferMode == 0) {
      shotsToFire = 0;
    }
  }

  switch (flywheelState) {
    case STATE_IDLE:
      if (triggerSwitch.isPressed() || revSwitch.isPressed()) {
        targetRPM = &revRPM;
        lastRevTime_ms = time_ms;
        flywheelState = STATE_ACCELERATING;
      } else if (time_ms < lastRevTime_ms + idleTime_ms &&
                 lastRevTime_ms > 0) {  // idle flywheels
        targetRPM = &idleRPM;
      } else {  // stop flywheels
        targetRPM = &zeroRPM;
      }
      break;

    case STATE_ACCELERATING:
      if (closedLoopFlywheels) {
        // If ALL motors are at target RPM update the blaster's state to
        // FULLSPEED. in the future add predictive capacity for when they'll be
        // up to speed in the future, taking into account pusher delay
        if (motorRPM[0] > firingRPM[0] &&
            (numMotors <= 1 || motorRPM[1] > firingRPM[1]) &&
            (numMotors <= 2 || motorRPM[2] > firingRPM[2]) &&
            (numMotors <= 3 || motorRPM[3] > firingRPM[3])) {
          flywheelState = STATE_FULLSPEED;
        }

      } else if (!closedLoopFlywheels &&
                 time_ms > lastRevTime_ms + firingDelay_ms) {
        flywheelState = STATE_FULLSPEED;
      }
      break;

    case STATE_FULLSPEED:
      if (!revSwitch.isPressed() && shotsToFire == 0 && !firing) {
        flywheelState = STATE_IDLE;
      } else if (shotsToFire > 0 || firing) {
        switch (pusherType) {
          case PUSHER_MOTOR_CLOSEDLOOP:
            cycleSwitch.update();
            if (shotsToFire > 0 && !firing) {  // start pusher stroke
              digitalWrite(pins.pusher, HIGH);
              digitalWrite(pins.pusherBrake, LOW);
              firing = true;
              pusherTimer_ms = time_ms;
            } else if (firing && shotsToFire == 0 &&
                       cycleSwitch.pressed()) {  // brake pusher
              digitalWrite(pins.pusher, HIGH);
              digitalWrite(pins.pusherBrake, HIGH);
              firing = false;
            } else if (firing && shotsToFire > 0 && cycleSwitch.released()) {
              shotsToFire = shotsToFire - 1;
              pusherTimer_ms = time_ms;
            } else if (firing &&
                       time_ms > pusherTimer_ms +
                                     pusherStallTime_ms) {  // stall protection
              digitalWrite(pins.pusher, LOW);               // let pusher coast
              digitalWrite(pins.pusherBrake, LOW);
              shotsToFire = 0;
              firing = false;
              Serial.println("Pusher motor stalled!");
            }
            break;

          case PUSHER_SOLENOID_OPENLOOP:
            if (shotsToFire > 0 && !firing &&
                time_ms > pusherTimer_ms +
                              solenoidRetractTime_ms) {  // extend solenoid
              digitalWrite(pins.pusher, HIGH);
              firing = true;
              shotsToFire -= 1;
              pusherTimer_ms = time_ms;
              Serial.println("solenoid extending");
            } else if (firing &&
                       time_ms >
                           pusherTimer_ms +
                               solenoidExtendTime_ms) {  // retract solenoid
              digitalWrite(pins.pusher, LOW);
              firing = false;
              pusherTimer_ms = time_ms;
              Serial.println("solenoid retracting");
            }
            break;
        }
      }
      break;
  }

  if (closedLoopFlywheels) {
    // --ray-- Andrew's control code goes here
  } else {  // open loop case
    for (int i = 0; i < numMotors; i++) {
      if (throttleValue[i] == 0) {
        throttleValue[i] =
            min(maxThrottle, maxThrottle * *targetRPM[i] / batteryADC_mv *
                                 1000 / scaledMotorKv);
      } else {
        throttleValue[i] =
            max(min(maxThrottle, maxThrottle * *targetRPM[i] / batteryADC_mv *
                                     1000 / scaledMotorKv),
                throttleValue[i] - spindownSpeed);
      }
    }
  }

  // send signal to ESCs
  if (dshotMode == DSHOT_OFF) {
    for (int i = 0; i < numMotors; i++) {
      servo[i].writeMicroseconds(throttleValue[i] / 2 + 1000);
    }
  } else {
    for (int i = 0; i < numMotors; i++) {
      dshot[i].send_dshot_value(throttleValue[i] + 48, NO_TELEMETRIC);
    }
  }
  ArduinoOTA.handle();
  loopTime_us = micros() - loopStartTimer_us;  // 'us' is microseconds
  if (loopTime_us > targetLoopTime_us) {
    Serial.print("loop over time, ");
    Serial.println(loopTime_us);
  } else {
    delayMicroseconds(max((long)(0), (long)(targetLoopTime_us - loopTime_us)));
  }
}

void WiFiInit() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid, wifiPass);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Connection Failed!");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PW);
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
        } else {  // U_SPIFFS
          type = "filesystem";
        }
        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS
        // using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() { Serial.println("\nEnd"); })
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

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
