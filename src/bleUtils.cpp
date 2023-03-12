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
    sSolenoidCharacteristic.setValue("\x0a\x07\x44\x65\x74\x6c\x61\x66\x66\x12\x0a\x08\x14\x10\x01\x18\x15\x20\x02\x28\x16\x1a\x45\x0a\x12\x0a\x0c\xb0\xea\x01\xb0\xea\x01\xb0\xea\x01\xb0\xea\x01\x10\x03\x18\x01\x0a\x12\x0a\x0c\xc0\xbb\x01\xc0\xbb\x01\xc0\xbb\x01\xc0\xbb\x01\x10\x01\x18\x02\x0a\x1b\x0a\x0c\xd0\x86\x03\xd0\x86\x03\xd0\x86\x03\xd0\x86\x03\x10\xff\xff\xff\xff\xff\xff\xff\xff\xff\x01\x18\x03\x22\x34\x08\x04\x10\x01\x18\x03\x20\x01\x2a\x09\x08\x8c\x15\x10\xc0\xbb\x01\x18\x50\x2a\x09\x08\x8c\x15\x10\xc0\xbb\x01\x18\x50\x2a\x09\x08\x8c\x15\x10\xc0\xbb\x01\x18\x50\x2a\x09\x08\x8c\x15\x10\xc0\xbb\x01\x18\x50\x3a\x0b\x08\x02\x12\x04\x08\x01\x10\x1b\x28\xdc\x56\x40\x04\x0a\x07\x44\x65\x74\x6c\x61\x66\x66\x12\x02\x08\x0f");
    sSolenoidCharacteristic.setCallbacks(new MyCallbacks);
    pSolenoid->start();

    // Start advertising
    pServer->getAdvertising()->start();
    shell.printf("BLE initialization finished\n");
}
