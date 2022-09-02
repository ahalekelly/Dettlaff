#include <Arduino.h>
#include <ArduinoOTA.h>
#define BOUNCE_LOCK_OUT // improves rev responsiveness at the risk of spurious signals from noise
#include "Bounce2.h"
#include "DShotRMT.h"
#include "ESP32Servo.h"

// Configuration Variables
uint16_t revThrottle = 1999; // scale is 0 - 1999
uint16_t idleThrottle = 50; // scale is 0 - 1999
uint32_t idleTime = 4000; // ms
bool revSwitchNormallyClosed = false; // should we invert rev signal?
bool triggerSwitchNormallyClosed = false;
bool cycleSwitchNormallyClosed = false;
uint16_t debounceTime = 50; // ms
char wifiSsid[32] = "ssid";
char wifiPass[63] = "pass";

// Advanced Configuration Variables
char AP_SSID[32] = "Dettlaff";
char AP_PW[32] = "KellyIndu";

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
} Pins_t;

const Pins_t pins_v0_3 = {
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
};

const Pins_t pins_v0_2 = {
  .revSwitch = 15,
  .esc1 = 19,
  .esc2 = 18,
  .esc3 = 5,
  .esc4 = 17,
  .telem = 16,
  .button = 0,
};

const Pins_t pins_v0_1 = {
  .revSwitch = 12,
  .esc1 = 4,
  .esc2 = 2,
  .esc3 = 15,
  .esc4 = 13,
};

Pins_t pins = pins_v0_3;
dshot_mode_t dshotMode =  DSHOT300; // DSHOT_OFF to fall back to servo PWM
uint16_t loopTime = 1000; // microseconds
// End Configuration Variables

uint32_t loopStartTime = micros();
uint32_t prevTime = micros();
uint16_t throttleValue = 0; // scale is 0 - 1999

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
  prevTime = loopStartTime;
  loopStartTime = micros();
  if (pins.revSwitch) {
    revSwitch.update();
  }
  if (pins.cycleSwitch) {
    cycleSwitch.update();
  }
  if (pins.triggerSwitch) {
    triggerSwitch.update();
  }
  if (loopStartTime > 5000000) { // for first 5s, send min throttle so ESCs can boot & arm
    if (pins.triggerSwitch) {
      if (triggerSwitch.isPressed()) {
        digitalWrite(pins.pusher, HIGH);
        digitalWrite(pins.pusherBrake, LOW);
      } else {
        digitalWrite(pins.pusher, HIGH);
        digitalWrite(pins.pusherBrake, HIGH);
      }
    }
    if (revSwitch.isPressed()) {
      throttleValue = revThrottle;
    } else if (revSwitch.currentDuration() < idleTime) {
      throttleValue = idleThrottle;
    } else {
      throttleValue = 0;
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
//  Serial.println(loopStartTime - prevTime);
  ArduinoOTA.handle();
  delayMicroseconds(max((long)(0), (long)(loopTime-(micros()-loopStartTime))));
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
  
    // No authentication by default
    // ArduinoOTA.setPassword("admin");
    
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
}
