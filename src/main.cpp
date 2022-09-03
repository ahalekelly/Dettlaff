#include <Arduino.h>
#include <ArduinoOTA.h>
#define BOUNCE_LOCK_OUT // improves rev responsiveness at the risk of spurious signals from noise
#include "Bounce2.h"
#include "DShotRMT.h"
#include "ESP32Servo.h"

// Configuration Variables
char wifiSsid[32] = "ssid";
char wifiPass[63] = "pass";
uint32_t revRPM = 50000;
uint32_t idleRPM = 1000;
uint32_t idleTime_ms = 5000;
uint32_t scaledMotorKv = 2550 * 11; // motor kv * battery voltage divider ratio
uint8_t burstLength = 3;
bool closedLoopControl = false;
uint16_t firingDelay_ms = 200;
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
} Pins_t;

const Pins_t pins_v0_3_n20 = {
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

const Pins_t pins_v0_3_noid = {
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

const Pins_t pins_v0_2 = {
  .revSwitch = 15,
  .esc1 = 19,
  .esc2 = 18,
  .esc3 = 5,
  .esc4 = 17,
  .telem = 16,
  .button = 0,
  .batteryADC = 12,
};

const Pins_t pins_v0_1 = {
  .revSwitch = 12,
  .esc1 = 4,
  .esc2 = 2,
  .esc3 = 15,
  .esc4 = 13,
};

Pins_t pins = pins_v0_3_n20;
// End Configuration Variables

// state machine diagram https://drive.google.com/file/d/1OglAILRt0AgflKg7DCO_BhukITWqTDnz/view?usp=sharing
enum state_t { // flywheel state, then pusher state
  STATE_IDLE,
  STATE_ACCELERATING_STANDBY,
  STATE_ACCELERATING_WAITING, // ACCELERATING = wheels not yet at full speed
  STATE_REV_STANDBY, // REV = wheels at full speed
  STATE_REV_FIRING,
  STATE_REV_RESET, // RESET = trigger is held down in semi or burst mode and cycle has completed
  STATE_IDLE_RESET
};

state_t state = STATE_IDLE;

uint32_t loopStartTimer_us = micros();
uint16_t loopTime_us = targetLoopTime_us;
uint32_t time_ms = millis();
int32_t lastRevTime_ms = -100000; // for calculating idling
uint32_t targetRPM = 0;
uint32_t throttleValue = 0; // scale is 0 - 1999
uint16_t batteryADC_mv = 0; // voltage at the ADC, after the voltage divider
uint8_t shotsToFire = 0;

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
  if (pins.cycleSwitch) {
    cycleSwitch.update();
  }
  if (pins.triggerSwitch) {
    triggerSwitch.update();
  }

  switch (state){
    case STATE_IDLE:
      if (triggerSwitch.isPressed()) {
        targetRPM = revRPM;
        lastRevTime_ms = time_ms;
        state = STATE_ACCELERATING_WAITING;
      } else if (revSwitch.isPressed()) {
        targetRPM = revRPM;
        lastRevTime_ms = time_ms;
        state = STATE_ACCELERATING_STANDBY;
      } else if (time_ms < lastRevTime_ms+idleTime_ms) { // idle flywheels
        targetRPM = idleRPM;
      } else { // stop flywheels
        targetRPM = 0;
      }
      break;
    case STATE_ACCELERATING_STANDBY:
      if (!closedLoopControl && time_ms > lastRevTime_ms + firingDelay_ms) {
        state = STATE_REV_STANDBY;
        if (triggerSwitch.isPressed()) {
          state =  STATE_REV_FIRING;
        }
      } else if (triggerSwitch.isPressed()) {
        state = STATE_ACCELERATING_WAITING;
      }
      break;
    case STATE_ACCELERATING_WAITING:
      if (!closedLoopControl && time_ms > lastRevTime_ms + firingDelay_ms) {
        state = STATE_REV_FIRING;
      }
      break;
    case STATE_REV_STANDBY:
      if (triggerSwitch.isPressed()) {
        state = STATE_REV_FIRING;
      } else if (!revSwitch.isPressed()) {
        state = STATE_IDLE;
      }
      break;
    case STATE_REV_FIRING:
      if (!triggerSwitch.isPressed() && cycleSwitch.isPressed()) {
        if (revSwitch.isPressed()) {
          state = STATE_REV_STANDBY;
        } else {
          state = STATE_IDLE;
        }
      }
      break;
    case STATE_REV_RESET:
      break;
    case STATE_IDLE_RESET:
      break;
  }

  if (state == STATE_REV_FIRING) {
    digitalWrite(pins.pusher, HIGH);
    digitalWrite(pins.pusherBrake, LOW);
  } else {
    digitalWrite(pins.pusher, HIGH);
    digitalWrite(pins.pusherBrake, HIGH);
  }

  if (closedLoopControl) {
    // ray control code goes here
  } else {
    throttleValue = max(maxThrottle * targetRPM / batteryADC_mv * 1000 / scaledMotorKv,
    throttleValue-spindownSpeed);
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
