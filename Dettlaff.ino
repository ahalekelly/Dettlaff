//#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#define BOUNCE_LOCK_OUT // improves rev responsiveness at the risk of spurious signals from noise
#include "src/Bounce2/src/Bounce2.h"
#include "arduino_secrets.h"

// Configuration Variables
uint16_t revThrottle = 1999; // scale is 0 - 1999
uint16_t idleThrottle = 50; // scale is 0 - 1999
uint32_t idleTime = 10000; // ms
bool revSwitchNormallyClosed = false; // should we invert rev signal?
uint16_t debounceTime = 50; // ms

// Advanced Configuration Variables
const uint16_t loopTime = 1000; // microseconds
const uint8_t revPin = 12; // Rev trigger
const uint8_t esc1Pin = 19; // Dettlaff Core v0.2
const uint8_t esc2Pin = 18; // Dettlaff Core v0.2
const uint8_t esc3Pin = 5; // Dettlaff Core v0.2
const uint8_t esc4Pin = 17; // Dettlaff Core v0.2
//const uint8_t esc1Pin = 4; // Dettlaff Core v0.1 - must comment out everything that uses ESC2!
//const uint8_t esc3Pin = 15; // Dettlaff Core v0.1
//const uint8_t esc4Pin = 13; // Dettlaff Core v0.1
//const uint8_t esc3Pin = 22; // Nodemcu dev board
//const uint8_t esc4Pin = 18; // Nodemcu dev board
#define DSHOT DSHOT300 //comment out to fall back to servo PWM
// End Configuration Variables

#ifdef DSHOT
  #include "src/DShotRMT/src/DShotRMT.h"
  DShotRMT dshot1(esc1Pin, RMT_CHANNEL_1);
  DShotRMT dshot2(esc2Pin, RMT_CHANNEL_2);
  DShotRMT dshot3(esc3Pin, RMT_CHANNEL_3);
  DShotRMT dshot4(esc4Pin, RMT_CHANNEL_4);
#else
  #include "src/ESP32Servo/src/ESP32Servo.h"
  Servo servo1;
  Servo servo2;
  Servo servo3;
  Servo servo4;
#endif

uint32_t loopStartTime = micros();
uint32_t prevTime = micros();
uint16_t throttleValue = 0; // scale is 0 - 1999
Bounce2::Button revSwitch = Bounce2::Button();

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  WiFiInit();
  revSwitch.attach(revPin, INPUT_PULLUP);
  revSwitch.interval(debounceTime);
  revSwitch.setPressedState(revSwitchNormallyClosed);
  #ifdef DSHOT
    dshot1.begin(DSHOT, false);  // bitrate & bidirectional
    dshot2.begin(DSHOT, false);
    dshot3.begin(DSHOT, false);
    dshot4.begin(DSHOT, false);
  #else
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    servo1.setPeriodHertz(200);
    servo2.setPeriodHertz(200);
    servo3.setPeriodHertz(200);
    servo4.setPeriodHertz(200);
    servo1.attach(esc1Pin);
    servo2.attach(esc2Pin);
    servo3.attach(esc3Pin);
    servo4.attach(esc4Pin);
  #endif
}

void loop() {
  prevTime = loopStartTime;
  loopStartTime = micros();
  ArduinoOTA.handle();
  revSwitch.update();
  if (loopStartTime > 5000000) { // for first 5s, send min throttle so ESCs can boot & arm
    if (revSwitch.isPressed()) {
      throttleValue = revThrottle;
    } else if (revSwitch.currentDuration() < idleTime) {
      throttleValue = idleThrottle;
    } else {
      throttleValue = 0;
    }
  }
  if (revSwitch.changed()) {
    Serial.print(revSwitch.isPressed());
    Serial.print(" ");
    Serial.println(throttleValue);
  }
  #ifdef DSHOT
    dshot1.send_dshot_value(throttleValue+48, NO_TELEMETRIC);
    dshot2.send_dshot_value(throttleValue+48, NO_TELEMETRIC);
    dshot3.send_dshot_value(throttleValue+48, NO_TELEMETRIC);
    dshot4.send_dshot_value(throttleValue+48, NO_TELEMETRIC);
  #else
    servo1.writeMicroseconds(throttleValue/2 + 1000);
    servo2.writeMicroseconds(throttleValue/2 + 1000);
    servo3.writeMicroseconds(throttleValue/2 + 1000);
    servo4.writeMicroseconds(throttleValue/2 + 1000);
  #endif
//  Serial.println(loopStartTime - prevTime);
  delayMicroseconds(max((long)(0), (long)(loopTime-(micros()-loopStartTime))));
}

void WiFiInit() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Connection Failed!");
  } else {
    Serial.print("WiFi Connected ");
    Serial.println(SSID);
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
