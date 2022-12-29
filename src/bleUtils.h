#ifndef __ble_utils_h_
#define __ble_utils_h_

#include <BLE2902.h> // Simplifies battery level broadcasting(?)
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// Enables shell commands
// clang-format off
/* For some strange reason SimpleSerialShell doesn't include Steam.h, so we have to include it before the first time it's included or the library breaks
 * This appears to be alphabetical by file and not by the parent files that invokes it, so just include stream.h everywhere to be safe
 */
#include <Stream.h>
#include <SimpleSerialShell.h>
// clang-format on
extern SimpleSerialShell& shell;

int shellCommandBattLevel(int argc, char** argv);

static BLEUUID sBatteryService((uint16_t)0x180F);
static BLECharacteristic sBatteryLevelCharacteristic(BLEUUID((uint16_t)0x2A19), (BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY));
static BLEDescriptor sBatteryLevelDescriptor(BLEUUID((uint16_t)0x2901));

static BLEUUID sSolenoidService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static constexpr char* cSolenoidCharacterisitcUUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
static BLECharacteristic sSolenoidCharacteristic(cSolenoidCharacterisitcUUID, (BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE));

static bool _BLEClientConnected = false;

// The solenoid extend time global from main.cpp
// I don't like it, but this is a workaround for now
extern uint16_t solenoidExtendTime_ms;

class BatteryServerCallbacks : public BLEServerCallbacks {
public:
    void onConnect(BLEServer* pServer)
    {
        shell.printf("Connected to BLE client\n");
        _BLEClientConnected = true;
    };

    void onDisconnect(BLEServer* pServer)
    {
        shell.printf("Disonnected from BLE client\n");
        _BLEClientConnected = false;
    }
};

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic)
    {
        std::string value = pCharacteristic->getValue();
        shell.printf("Got updated value %s\n", value);

        if (value.length() > 0) {
            int newVal = atoi(value.c_str());

            if (newVal >= 0) {
                shell.printf("Updating solenoid extend time to %d\n", newVal);
                solenoidExtendTime_ms = newVal;
            }
        }
    }
};

void InitBLE();

inline void BLESetBattLevel(uint8_t level)
{
    sBatteryLevelCharacteristic.setValue(&level, 1);
    sBatteryLevelCharacteristic.notify();
}

#endif //__ble_utils_h_
