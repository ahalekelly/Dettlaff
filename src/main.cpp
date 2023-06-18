#include <Arduino.h>
#include <ArduinoOTA.h>
#define BOUNCE_LOCK_OUT // improves trigger responsiveness at the risk of spurious signals from noise
#include "Bounce2.h"
#include "DShotRMT.h"
#include "ESP32Servo.h"
#include "types.h"
#include "CONFIGURATION.h"
#include "driver.h"
#include "fetDriver.h"
#include "at8870Driver.h"
#include "hBridgeDriver.h"

uint32_t loopStartTimer_us = micros();
uint16_t loopTime_us = targetLoopTime_us;
uint32_t time_ms = millis();
uint32_t lastRevTime_ms = 0; // for calculating idling
uint32_t pusherTimer_ms = 0;
uint32_t zeroRPM[4] = {0, 0, 0, 0};
uint32_t (*targetRPM)[4]; // a pointer to a uint32_t[4] array. always points to either revRPM, idleRPM, or zeroRPM
uint32_t throttleValue[4] = {0, 0, 0, 0}; // scale is 0 - 1999
uint16_t shotsToFire = 0;
flywheelState_t flywheelState = STATE_IDLE;
bool firing = false;
bool closedLoopFlywheels = false;
uint32_t scaledMotorKv = motorKv * 11; // motor kv * battery voltage resistor divider ratio
const uint32_t maxThrottle = 1999;
uint32_t motorRPM[4] = {0, 0, 0, 0};
Driver* pusher;

Bounce2::Button revSwitch = Bounce2::Button();
Bounce2::Button triggerSwitch = Bounce2::Button();
Bounce2::Button cycleSwitch = Bounce2::Button();
Bounce2::Button button = Bounce2::Button();

// Declare servo variables for each motor. 
Servo servo[4];
DShotRMT dshot[4] = {DShotRMT(pins.esc1, RMT_CHANNEL_1), DShotRMT(pins.esc2, RMT_CHANNEL_2),
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

  switch (pins.pusherDriverType) {
    case HBRIDGE_DRIVER:
      pusher = new Hbridge(pins.pusher1H, pins.pusher1L, pins.pusher2H, pins.pusher2L, maxDutyCycle_pct, pwmFreq_hz, deadtime);
      break;
    case AT8870_DRIVER:
      pusher = new At8870(pins.pusher1L, pins.pusher2L, pwmFreq_hz);
      break;
    case FET_DRIVER:
      pusher = new Fet(pins.pusher1H);
      break;
    default:
      break;
  }

  if (pins.revSwitch) {
    revSwitch.attach(pins.revSwitch, INPUT_PULLUP);
    revSwitch.interval(debounceTime_ms);
    revSwitch.setPressedState(revSwitchNormallyClosed);
  }
  if (pins.triggerSwitch) {
    triggerSwitch.attach(pins.triggerSwitch, INPUT_PULLUP);
    triggerSwitch.interval(debounceTime_ms);
    triggerSwitch.setPressedState(triggerSwitchNormallyClosed);
  }
  if (pins.cycleSwitch) {
    cycleSwitch.attach(pins.cycleSwitch, INPUT_PULLUP);
    cycleSwitch.interval(debounceTime_ms);
    cycleSwitch.setPressedState(cycleSwitchNormallyClosed);
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
      dshot[i].begin(dshotMode, false); // bitrate & bidirectional
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
        targetRPM = &revRPM;
        lastRevTime_ms = time_ms;
        flywheelState = STATE_ACCELERATING;
      } else if (time_ms < lastRevTime_ms + idleTime_ms && lastRevTime_ms > 0) { // idle flywheels
        targetRPM = &idleRPM;
      } else { // stop flywheels
        targetRPM = &zeroRPM;
      }
      break;

    case STATE_ACCELERATING:
      if (closedLoopFlywheels) {
        // If ALL motors are at target RPM update the blaster's state to FULLSPEED.
        // in the future add predictive capacity for when they'll be up to speed in the future, taking into account pusher delay
        if ( motorRPM[0] > firingRPM[0] &&
        (numMotors <= 1 || motorRPM[1] > firingRPM[1]) &&
        (numMotors <= 2 || motorRPM[2] > firingRPM[2]) &&
        (numMotors <= 3 || motorRPM[3] > firingRPM[3]) ) {
          flywheelState = STATE_FULLSPEED;
        }

      } else if (!closedLoopFlywheels && time_ms > lastRevTime_ms + firingDelay_ms) {
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
              pusher->drive(100, pusherReverseDirection);
              firing = true;
              pusherTimer_ms = time_ms;
            } else if (firing && cycleSwitch.pressed()) {
              shotsToFire = shotsToFire - 1;
              if (shotsToFire == 0 ) {  // brake pusher
                pusher->brake();
                firing = false;
              }
              pusherTimer_ms = time_ms;
            } else if (firing && time_ms > pusherTimer_ms + pusherStallTime_ms) { // stall protection
              pusher->coast();
              shotsToFire = 0;
              firing = false;
              Serial.println("Pusher motor stalled!");
            }
            break;

          case PUSHER_SOLENOID_OPENLOOP:
            if (shotsToFire > 0 && !firing && time_ms > pusherTimer_ms + solenoidRetractTime_ms) { // extend solenoid
              pusher->drive(100, pusherReverseDirection);
              firing = true;
              shotsToFire -= 1;
              pusherTimer_ms = time_ms;
              Serial.println("solenoid extending");
            } else if (firing && time_ms > pusherTimer_ms + solenoidExtendTime_ms) { // retract solenoid
              pusher->coast();
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
    // PID control code goes here
  } else { // open loop case
    for (int i = 0; i < numMotors; i++) {
      if (throttleValue[i] == 0) {
        throttleValue[i] = min(maxThrottle, maxThrottle * (*targetRPM)[i] / batteryADC_mv * 1000 / scaledMotorKv);
      } else {
        throttleValue[i] = max(min(maxThrottle, maxThrottle * (*targetRPM)[i] / batteryADC_mv * 1000 / scaledMotorKv),
        throttleValue[i]-spindownSpeed);}
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
  loopTime_us = micros() - loopStartTimer_us; // 'us' is microseconds
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
