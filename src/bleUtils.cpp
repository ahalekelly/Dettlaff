#include "bleutils.h"

/**************************************************************/
/******************* Shell Command Battery ********************/
/**************************************************************/

enum cCommandPositions {
    cCommand,
    cFunction,
    cArg,
};

static constexpr size_t cMaxArgLen = strlen("getExtendTime");

int shellCommandBattLevel(int argc, char** argv)
{
    int ret = 0;

    if (strncmp(argv[cFunction], "send", cMaxArgLen) == 0) {
        if (argc != cArg + 1) {
            shell.printf("Incorrect number of arguments; command must be in format `battery send <number between 0-100>`\n");
            ret - 1;
        }
        int battLevel = atoi(argv[cArg]);
        if (battLevel < 0) {
            shell.printf("Arguement is not a number; command must be in format `battery send <number between 0-100>`\n");
            ret = -1;
            return ret;
        } else if (battLevel > 100) {
            shell.printf("Arguement must be between 0 and 100; command must be in format `battery send <number between 0-100>`\n");
            ret = -1;
            return ret;
        }

        BLESetBattLevel(battLevel);
        shell.printf("Sent battery level of %d\n", atoi(argv[cArg]));
    } else if (strncmp(argv[cFunction], "help", cMaxArgLen) == 0) {
        shell.printf("Command must be in format `battery send <number between 0-100>\n");
    }

    return ret;
}

void InitBLE()
{
    BLEDevice::init("Detlaff");

    // Create the BLE Server
    BLEServer* pServer = BLEDevice::createServer();
    pServer->setCallbacks(new BatteryServerCallbacks());

    // Create the BLE Service
    BLEService* pBattery = pServer->createService(sBatteryService);
    pBattery->addCharacteristic(&sBatteryLevelCharacteristic);
    sBatteryLevelDescriptor.setValue("Percentage 0 - 100");
    sBatteryLevelCharacteristic.addDescriptor(&sBatteryLevelDescriptor);
    sBatteryLevelCharacteristic.addDescriptor(new BLE2902());
    pServer->getAdvertising()->addServiceUUID(sBatteryService);
    pBattery->start();

    BLEService* pSolenoid = pServer->createService(sSolenoidService);
    pSolenoid->addCharacteristic(&sSolenoidCharacteristic);
    pServer->getAdvertising()->addServiceUUID(sSolenoidService);
    pServer->getAdvertising()->setScanResponse(true);
    sSolenoidCharacteristic.setValue("22");
    sSolenoidCharacteristic.setCallbacks(new MyCallbacks);
    pSolenoid->start();

    // Start advertising
    pServer->getAdvertising()->start();
    shell.printf("BLE initialization finished\n");
}
