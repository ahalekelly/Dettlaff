#include <Arduino.h>
#include <ArduinoOTA.h>
#define BOUNCE_LOCK_OUT // improves rev responsiveness at the risk of spurious signals from noise
#include "Bounce2.h"
#include "DShotRMT.h"
#include "ESP32Servo.h"
#include "types.h"

// Configuration Variables
char wifiSsid[32] = "ssid";
char wifiPass[63] = "pass";
uint32_t revRPM = 50000;
uint32_t idleRPM = 1000;
uint32_t idleTime_ms = 5000;
uint32_t scaledMotorKv = 2550 * 11; // motor kv * battery voltage resistor divider ratio
uint16_t burstLength = 3;
uint8_t bufferMode = 0;
// 0 = stop firing when trigger is released
// 1 = complete current burst
// 2 = fire as many bursts as trigger pulls
// for full auto, set burstlength high (100) and bufferMode = 0
bool closedLoopFlywheels = false;
uint16_t firingDelay_ms = 200;
pusherType_t pusherType = PUSHER_MOTOR_CLOSEDLOOP;
uint16_t pusherStallTime_ms = 200;
uint16_t solenoidExtendTime_ms = 100;
uint16_t solenoidRetractTime_ms = 100;
uint16_t spindownSpeed = 1;
bool revSwitchNormallyClosed = false; // should we invert rev signal?
bool triggerSwitchNormallyClosed = false;
bool cycleSwitchNormallyClosed = false;
uint16_t debounceTime = 50; // ms

// Advanced Configuration Variables
char AP_SSID[32] = "Dettlaff";
char AP_PW[32] = "KellyIndu";
dshot_mode_t dshotMode =  DSHOT300; // DSHOT_OFF to fall back to servo PWM
uint16_t targetLoopTime_us = 1000; // microseconds

const pins_t pins_v0_3_n20 = {
  .revSwitch = 15,
  .triggerSwitch = 32,
  .cycleSwitch = 23,
  .flywheel = 2,
  .pusher = 12,
  .pusherBrake = 13,
  .esc1 = 19,
  .esc2 = 18,
  .esc3 = 5,
  .esc4 = 17,
  .telem = 16,
  .button = 0,
  .batteryADC = 33,
};

const pins_t pins_v0_3_noid = {
  .revSwitch = 15,
  .triggerSwitch = 32,
  .pusher = 2,
  .esc1 = 19,
  .esc2 = 18,
  .esc3 = 5,
  .esc4 = 17,
  .telem = 16,
  .button = 0,
  .batteryADC = 33,
};

const pins_t pins_v0_2 = {
  .revSwitch = 15,
  .esc1 = 19,
  .esc2 = 18,
  .esc3 = 5,
  .esc4 = 17,
  .telem = 16,
  .button = 0,
  .batteryADC = 12,
};

const pins_t pins_v0_1 = {
  .revSwitch = 12,
  .esc1 = 4,
  .esc2 = 2,
  .esc3 = 15,
  .esc4 = 13,
};

pins_t pins = pins_v0_3_n20;

// End Configuration Variables

uint32_t loopStartTimer_us = micros();
uint16_t loopTime_us = targetLoopTime_us;
uint32_t time_ms = millis();
uint32_t lastRevTime_ms = 0; // for calculating idling
uint32_t pusherTimer_ms = 0;
uint32_t targetRPM = 0;
uint32_t throttleValue = 0; // scale is 0 - 1999
uint32_t batteryADC_mv = 1000; // voltage at the ADC, after the voltage divider
uint16_t shotsToFire = 0;
flywheelState_t flywheelState = STATE_IDLE;
bool firing = false;

const uint32_t maxThrottle = 1999;

Bounce2::Button revSwitch = Bounce2::Button();
Bounce2::Button triggerSwitch = Bounce2::Button();
Bounce2::Button cycleSwitch = Bounce2::Button();
Bounce2::Button button = Bounce2::Button();

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

DShotRMT dshot1(pins.esc1, RMT_CHANNEL_1);
DShotRMT dshot2(pins.esc2, RMT_CHANNEL_2);
DShotRMT dshot3(pins.esc3, RMT_CHANNEL_3);
DShotRMT dshot4(pins.esc4, RMT_CHANNEL_4);

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
    pinMode(pins.pusherBrake, OUTPUT);
  }
  if (dshotMode == DSHOT_OFF) {
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    servo1.setPeriodHertz(200);
    servo2.setPeriodHertz(200);
    servo3.setPeriodHertz(200);
    servo4.setPeriodHertz(200);
    servo1.attach(pins.esc1);
    servo2.attach(pins.esc2);
    servo3.attach(pins.esc3);
    servo4.attach(pins.esc4);
  } else {
    dshot1.begin(dshotMode, false);  // bitrate & bidirectional
    dshot2.begin(dshotMode, false);
    dshot3.begin(dshotMode, false);
    dshot4.begin(dshotMode, false);
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

  if (triggerSwitch.pressed()) { // pressed and released are transitions, isPressed is for state
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

  switch (flywheelState){

    case STATE_IDLE:
      if (triggerSwitch.isPressed() || revSwitch.isPressed()) {
        targetRPM = revRPM;
        lastRevTime_ms = time_ms;
        flywheelState = STATE_ACCELERATING;
      } else if (time_ms < lastRevTime_ms + idleTime_ms && lastRevTime_ms > 0) { // idle flywheels
        targetRPM = idleRPM;
      } else { // stop flywheels
        targetRPM = 0;
      }
      break;

    case STATE_ACCELERATING:
      if ((closedLoopFlywheels)
      || (!closedLoopFlywheels && time_ms > lastRevTime_ms + firingDelay_ms)) {
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
            if (shotsToFire > 0 && !firing) { // start pusher stroke
              digitalWrite(pins.pusher, HIGH);
              digitalWrite(pins.pusherBrake, LOW);
              firing = true;
              pusherTimer_ms = time_ms;
            } else if (firing && shotsToFire == 0 && cycleSwitch.pressed()) { // brake pusher
              digitalWrite(pins.pusher, HIGH);
              digitalWrite(pins.pusherBrake, HIGH);
              firing = false;
            } else if (firing && shotsToFire > 0 && cycleSwitch.released()) {
              shotsToFire = shotsToFire-1;
              pusherTimer_ms = time_ms;
            } else if (firing && time_ms > pusherTimer_ms + pusherStallTime_ms) { // stall protection
              digitalWrite(pins.pusher, LOW); // let pusher coast
              digitalWrite(pins.pusherBrake, LOW);
              shotsToFire = 0;
              firing = false;
              Serial.println("Pusher motor stalled!");
            }
            break;

          case PUSHER_SOLENOID_OPENLOOP:
            if (shotsToFire > 0 && !firing && time_ms > pusherTimer_ms + solenoidRetractTime_ms) { // extend solenoid
              digitalWrite(pins.pusher, HIGH);
              firing = true;
              shotsToFire -= 1;
              pusherTimer_ms = time_ms;
            } else if (firing && time_ms > pusherTimer_ms + solenoidExtendTime_ms) { // retract solenoid
              digitalWrite(pins.pusher, LOW);
              firing = false;
              pusherTimer_ms = time_ms;
            }
            break;
        }
      }
      break;
  }

  if (closedLoopFlywheels) {
    // ray control code goes here
  } else {
    if (throttleValue == 0) {
      throttleValue = min(maxThrottle, maxThrottle * targetRPM / batteryADC_mv * 1000 / scaledMotorKv);
    } else {
      throttleValue = max(min(maxThrottle, maxThrottle * targetRPM / batteryADC_mv * 1000 / scaledMotorKv),
      throttleValue-spindownSpeed);
    }
  }

  // send signal to ESCs
  if (dshotMode == DSHOT_OFF) {
    servo1.writeMicroseconds(throttleValue/2 + 1000);
    servo2.writeMicroseconds(throttleValue/2 + 1000);
    servo3.writeMicroseconds(throttleValue/2 + 1000);
    servo4.writeMicroseconds(throttleValue/2 + 1000);
  } else {
    dshot1.send_dshot_value(throttleValue+48, NO_TELEMETRIC);
    dshot2.send_dshot_value(throttleValue+48, NO_TELEMETRIC);
    dshot3.send_dshot_value(throttleValue+48, NO_TELEMETRIC);
    dshot4.send_dshot_value(throttleValue+48, NO_TELEMETRIC);
  }
  ArduinoOTA.handle();
  loopTime_us = micros() - loopStartTimer_us;
  if (loopTime_us > targetLoopTime_us) {
    Serial.print("loop over time, ");
    Serial.println(loopTime_us);
  } else {
    delayMicroseconds(max((long)(0), (long)(targetLoopTime_us-loopTime_us)));
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
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  
    ArduinoOTA.begin();
  
    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
