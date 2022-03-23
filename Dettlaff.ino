#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#define BOUNCE_LOCK_OUT // improves rev responsiveness at the risk of spurious signals from noise
#include <Bounce2.h>
#include "src/DShotRMT/src/DShotRMT.h"
#include "arduino_secrets.h"

// Configuration Variables
uint16_t revThrottle = 2047; // scale is 48 - 2047
uint16_t idleThrottle = 100; // scale is 48 - 2047
uint32_t idleTime = 10000; // ms
bool revSwitchNormallyClosed = false; // should we invert rev signal?
uint16_t debounceTime = 50; // ms 

// Advanced Configuration Variables
const uint16_t loopTime = 1000; // microseconds
const uint8_t revPin = 12;
DShotRMT dshot3(GPIO_NUM_15, RMT_CHANNEL_3); // ESC3
DShotRMT dshot4(GPIO_NUM_13, RMT_CHANNEL_4); // ESC4

// End Configuration Variables

uint32_t loopStartTime = micros();
uint32_t prevTime = micros();
uint16_t throttleValue = 48;  // throttle range is 48 to 2047
Bounce2::Button revSwitch = Bounce2::Button();

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  WiFiInit();
  revSwitch.attach(revPin, INPUT_PULLUP);
  revSwitch.interval(debounceTime);
  revSwitch.setPressedState(revSwitchNormallyClosed);
  dshot3.begin(DSHOT300, false);  // bitrate & bidirectional
  dshot4.begin(DSHOT300, false);
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
      throttleValue = 48;
    }
  }
  if (revSwitch.changed()) {
    Serial.print(revSwitch.isPressed());
    Serial.print(" ");
    Serial.println(throttleValue);
  }
  dshot3.send_dshot_value(throttleValue, NO_TELEMETRIC);
  dshot4.send_dshot_value(throttleValue, NO_TELEMETRIC);
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
