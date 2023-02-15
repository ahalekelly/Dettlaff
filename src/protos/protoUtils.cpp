#include "protoUtils.h"

#include "test.h"
// #include <memory>

// using namespace NanoPb::Converter;

// class BlasterConverter : public MessageConverter<BlasterConverter, Blaster, BLASTER_Blaster, &BLASTER_Blaster_msg> {
// public:
//     static ProtoType encoderInit(const LocalType& local)
//     {
//         return ProtoType { .blasterName = StringConverter::encoderInit(local.blasterName) };
//     }

//     static ProtoType decoderInit(LocalType& local)
//     {
//         return ProtoType { .blasterName = StringConverter::decoderInit(local.blasterName) };
//     }

//     static bool decoderApply(const ProtoType& proto, LocalType& local) { return true; }
// };

// extern "C" {
void TestProtos(void)
{
    unsigned bufferLen = 128;
    uint8_t buffer[bufferLen];
    bool status;
    Blaster test1 = Blaster_init_zero;

    pb_ostream_t outputStream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    char name[] = "Detlaff";

    strncpy(test1.blasterName, name, 20);
    test1.has_controlconfig = true;
    test1.controlconfig.triggerSwitchPin = 15;

    status = pb_encode(&outputStream, Blaster_fields, &test1);
    assert(status);
    size_t messageLength = outputStream.bytes_written;

    shell.printf("Proto Buffer is: |0x");
    for (int ndx = 0; ndx < messageLength; ndx++) {
        shell.printf(" %02x", buffer[ndx]);
    }
    shell.printf("|\n");

    std::string bufferString = std::string((char*)buffer, messageLength);
    Blaster received = Blaster_init_zero;

    ProtoDecodeFromString(&received, bufferString);

    assert(strncmp(test1.blasterName, received.blasterName, 20) == 0);
}

void ProtoDecodeFromString(Blaster* received, const std::string& input)
{
    pb_istream_t inStream = pb_istream_from_buffer((const pb_byte_t*)input.c_str(), input.length());

    bool status = pb_decode(&inStream, Blaster_fields, received);
    assert(status);

    shell.printf("Received blaster:\n");
    PrettyPrintBlaster(shell, *received);
}

void PrettyPrintHardwareVersion(SimpleSerialShell& printer, Blaster& blaster)
{
    printer.printf("Detlaff Board Version: %s\n",
        ((blaster.hardwareVersion == HardwareVersion_VERSION_0P1)          ? "0.1"
                : (blaster.hardwareVersion == HardwareVersion_VERSION_0P2) ? "0.2"
                : (blaster.hardwareVersion == HardwareVersion_VERSION_0P3) ? "0.3"
                : (blaster.hardwareVersion == HardwareVersion_VERSION_0P4) ? "0.4"
                                                                           : "Invalid"));
}

void PrettyPrintControlConfig(SimpleSerialShell& printer, Blaster& blaster)
{
    if (blaster.has_controlconfig) {
        printer.printf("Control Config\n");
        ControlConfig controlConfig = blaster.controlconfig;
        printer.printf("\tTrigger Switch Pin: %d\n", controlConfig.triggerSwitchPin);

        printer.printf("\tTrigger Switch Orientation: ");
        switch (controlConfig.TriggerSwitchOrientation) {
        case SwitchOrientation_SWITCH_NORMALLY_OPEN:
            printer.printf("Normally Open");
            break;
        case SwitchOrientation_SWITCH_NORMALLY_CLOSED:
            printer.printf("Normally Closed");
            break;
        case SwitchOrientation_SWITCH_ORIENTATION_INVALID:
            // Fall through to default
        default:
            printer.printf("Invalid");
            break;
        }
        printer.printf("\n");

        printer.printf("\tRev Switch Pin: %d\n", controlConfig.optRevSwitchPin);

        printer.printf("\tRev Switch Orientation: ");
        switch (controlConfig.optRevSwitchOrientation) {
        case SwitchOrientation_SWITCH_NORMALLY_OPEN:
            printer.printf("Normally Open");
            break;
        case SwitchOrientation_SWITCH_NORMALLY_CLOSED:
            printer.printf("Normally Closed");
            break;
        case SwitchOrientation_SWITCH_ORIENTATION_INVALID:
            // Fall through to default
        default:
            printer.printf("Invalid");
            break;
        }
        printer.printf("\n");

        printer.printf("\tFire Mode Switch Pin: %d\n", controlConfig.fireModeCycleButtonPin);
    } else {
        printer.printf("No Control Config\n");
    }
}

void PrettyPrintControlParams(SimpleSerialShell& printer, Blaster& blaster)
{
    if (blaster.has_controlParams) {
        ControlParams controlParams = blaster.controlParams;
        printer.printf("Control Params\n");

        printer.printf("\tFiringModes:\n");
        if (controlParams.fireModesArray_count == 0) {
            printer.printf("\t\tWarning: No Firing modes");
        }
        for (int ndx = 0; ndx < controlParams.fireModesArray_count; ndx++) {
            FireModeEntry fireModeEntry = controlParams.fireModesArray[ndx];

            printer.printf("\tEntry %d", ndx);
            printer.printf("\t\tRpm: %d, Burst Length %d, Burst Type: ", fireModeEntry.rpm, fireModeEntry.burstLength);
            switch (fireModeEntry.type) {
            case PusherBurstType_PUSHER_BURST_STOP:
                printer.printf("Stop");
                break;
            case PusherBurstType_PUSHER_BURST_COMPLETE:
                printer.printf("Complete");
                break;
            case PusherBurstType_PUSHER_BURST_COUNT:
                printer.printf("Count");
                break;
            case PusherBurstType_PUSHER_BURST_INVALID:
            // Fall through to default
            default:
                printer.printf("Invalid");
                break;
            }
            printer.printf("\n");
        }
    } else {
        printer.printf("No Control Params\n");
    }
}

void PrettyPrintFlywheelConfig(SimpleSerialShell& printer, Blaster& blaster)
{
    if (blaster.has_flywheelConfig) {
        printer.printf("Flywheel Config\n");
        FlywheelConfig flywheelConfig = blaster.flywheelConfig;

        printer.printf("\tMotor Control Method: %s\n",
            ((flywheelConfig.controlMethod == MotorControlMethod_ESC_DSHOT)            ? "DShot"
                    : (flywheelConfig.controlMethod == MotorControlMethod_ESC_PWM)     ? "PWM"
                    : (flywheelConfig.controlMethod == MotorControlMethod_BRUSHED_PWM) ? "Brushed PWM"
                    : (flywheelConfig.controlMethod == MotorControlMethod_BRUSHED_PWM) ? "Brushed Binary"
                                                                                       : "Invalid"));

        printer.printf("\tDShot %s\n",
            ((flywheelConfig.dshotMode == DshotModes_DSHOT_150)           ? "150"
                    : (flywheelConfig.dshotMode == DshotModes_DSHOT_300)  ? "300"
                    : (flywheelConfig.dshotMode == DshotModes_DSHOT_600)  ? "600"
                    : (flywheelConfig.dshotMode == DshotModes_DSHOT_1200) ? "1200"
                                                                          : "Off"));

        printer.printf("\tTelemetry Method %s\n",
            ((flywheelConfig.telemetryMethod == TelemMethod_BIDIRECTIONAL_DSHOT) ? "DSHOT"
                    : (flywheelConfig.telemetryMethod == TelemMethod_SERIAL)     ? "Serial"
                    : (flywheelConfig.telemetryMethod == TelemMethod_TACHOMETER) ? "Techometer"
                                                                                 : "No Telemetry"));

        printer.printf("\tMotor Configs:\n");
        if (flywheelConfig.motorConfig_count == 0) {
            printer.printf("\t\tWarning: No Motors Configured");
        }
        for (int ndx = 0; ndx < flywheelConfig.motorConfig_count; ndx++) {
            MotorConfig motorConfig = flywheelConfig.motorConfig[ndx];

            printer.printf("Motor %d: KV: %d, Rev RPM: %d, Idle RPM %d, Firing Threshold RPM %d\n",
                ndx, motorConfig.motorKv, motorConfig.revRPM, motorConfig.idleRPM, motorConfig.firingThresholdRpm);
        }
    } else {
        printer.printf("No Flywheel Config\n");
    }
}
void PrettyPrintBlaster(SimpleSerialShell& printer, Blaster& blaster)
{
    printer.printf("Name: %s\n", blaster.blasterName);
    PrettyPrintHardwareVersion(printer, blaster);

    PrettyPrintControlConfig(printer, blaster);
    PrettyPrintControlParams(printer, blaster);
    PrettyPrintFlywheelConfig(printer, blaster);
}

// Do I need to past this by reference?
void ProtoEncodeToString(Blaster& received, std::string* output)
{

    // Do we need this intermediary buffer? I don't think we can directly interact with the cstring from ble
    // We also kinda need to interface with the BLE library in case we need to send a message longer than 512
    // But blaster shouldn't be longer than 512; the longer stuff like OTA, esc flashing, etc will probably have its own characteristics
    // TL;DR: This may not work well as a proto-only helper function
    unsigned bufferLen = 512;
}

/* Attempt with NanoPB_cpp*/
// void TestProtos(void)
// {
//     const Blaster test1({ "Detlaff" });

//     NanoPb::StringOutputStream outputStream;

//     assert(NanoPb::encode<BlasterConverter>(outputStream, test1));

//     // auto inputStream = NanoPb::StringInputStream(outputStream.release());

//     shell.printf("This should be here\n");
//     std::string oString = *outputStream.release().get();
//     shell.printf("Protobuf: %d, %s\n|0x", oString.length(), oString.c_str());
//     for (int i = 0; i < oString.length(); i++) {
//         shell.printf("%02x ", oString[i]);
//     }
//     shell.printf("|\n");
//     // Blaster received;

//     // assert(NanoPb::decode<BlasterConverter>(inputStream, received));

//     // assert(test1.blasterName.compare(received.blasterName) == 0);

//     std::string bleTest = "␇Detlaff";
//     Blaster received;

//     ProtoDecodeBleCharacteristic(&received, bleTest);
//     assert(test1.blasterName.compare(received.blasterName) == 0);
// }

// void ProtoDecodeBleCharacteristic(Blaster* received, std::string characteristicContents)
// {
//     // pb_ostream_t bleOStream = pb_ostream_from_buffer((pb_byte_t*)characteristicContents.c_str(), characteristicContents.length());
//     auto input = new NanoPb::BufferPtr(characteristicContents);
//     auto bleStream = NanoPb::StringInputStream(input);

//     NanoPb::decode<BlasterConverter>(bleStream, *received);

//     shell.printf("Received blaster with name: %s\n", received->blasterName.c_str());
// }
