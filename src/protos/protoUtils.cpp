#include "protoUtils.h"

#include <stdio.h>

#include "test.h"

const char defaultName[] = "Detlaff";

static void PrettyPrintHardwareVersion(SimpleSerialShell& printer, Blaster& blaster);
static void PrettyPrintControlConfig(SimpleSerialShell& printer, Blaster& blaster);
static void PrettyPrintControlParams(SimpleSerialShell& printer, Blaster& blaster);
static void PrettyPrintFlywheelConfig(SimpleSerialShell& printer, Blaster& blaster);
static void PrettyPrintFlywheelParams(SimpleSerialShell& printer, Blaster& blaster);
static void prettyPrintPusherConfig(SimpleSerialShell& printer, Blaster& blaster);

int ProtoDecodeFromString(Blaster* received, const std::string& input)
{
    pb_istream_t inStream = pb_istream_from_buffer((const pb_byte_t*)input.c_str(), input.length());

    return (pb_decode(&inStream, Blaster_fields, received)) ? EXIT_SUCCESS : EXIT_FAILURE;
}

int ProtoEncodeToString(const Blaster& received, std::string* output)
{
    const unsigned bufferLen = 512;
    uint8_t buffer[bufferLen];

    pb_ostream_t outputStream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    if (!pb_encode(&outputStream, Blaster_fields, &received)) {
        return EXIT_FAILURE;
    }

    output->insert(0, (char*)buffer, outputStream.bytes_written);

    return EXIT_SUCCESS;
}

void PrettyPrintBlaster(SimpleSerialShell& printer, Blaster& blaster)
{
    printer.printf("Name: %s\n", blaster.blasterName);
    PrettyPrintHardwareVersion(printer, blaster);

    PrettyPrintControlConfig(printer, blaster);
    PrettyPrintControlParams(printer, blaster);
    PrettyPrintFlywheelConfig(printer, blaster);
    PrettyPrintFlywheelParams(printer, blaster);
    prettyPrintPusherConfig(printer, blaster);
}

static Blaster getAllFieldsBlaster(void)
{
    Blaster allFields;

    ControlConfig controlConfig;
    controlConfig.triggerSwitchPin = 20; // Making up numbers here
    controlConfig.TriggerSwitchOrientation = SwitchOrientation_SWITCH_NORMALLY_OPEN;
    controlConfig.optRevSwitchPin = 21;
    controlConfig.optRevSwitchOrientation = SwitchOrientation_SWITCH_NORMALLY_CLOSED;
    controlConfig.fireModeCycleButtonPin = 22;

    FireModeEntry entry1;
    entry1.rpm_count = 4;
    entry1.rpm[0] = 30000;
    entry1.rpm[1] = 30000;
    entry1.rpm[2] = 30000;
    entry1.rpm[3] = 30000;
    entry1.burstLength = 3,
    entry1.type = PusherBurstType_PUSHER_BURST_STOP;

    FireModeEntry entry2;
    entry2.rpm_count = 4;
    entry2.rpm[0] = 24000;
    entry2.rpm[1] = 24000;
    entry2.rpm[2] = 24000;
    entry2.rpm[3] = 24000;
    entry2.burstLength = 1,
    entry2.type = PusherBurstType_PUSHER_BURST_COMPLETE;

    FireModeEntry entry3;
    entry3.rpm_count = 4;
    entry3.rpm[0] = 50000;
    entry3.rpm[1] = 50000;
    entry3.rpm[2] = 50000;
    entry3.rpm[3] = 50000;
    entry3.burstLength = -1,
    entry3.type = PusherBurstType_PUSHER_BURST_COUNT;

    ControlParams controlParams;
    controlParams.fireModesArray_count = 3;
    memcpy(&controlParams.fireModesArray[0], &entry1, sizeof(FireModeEntry));
    memcpy(&controlParams.fireModesArray[1], &entry2, sizeof(FireModeEntry));
    memcpy(&controlParams.fireModesArray[2], &entry3, sizeof(FireModeEntry));
    controlParams.currentFiringMode = 0;

    FlywheelConfig flywheelConfig;
    flywheelConfig.numMotors = 4;
    flywheelConfig.controlMethod = MotorControlMethod_ESC_DSHOT;
    flywheelConfig.dshotMode = DshotModes_DSHOT_600;
    flywheelConfig.telemetryMethod = TelemMethod_BIDIRECTIONAL_DSHOT;
    flywheelConfig.motorConfig_count = 4;
    flywheelConfig.motorConfig[0].motorKv = 2700;
    flywheelConfig.motorConfig[0].idleRPM = 24000;
    flywheelConfig.motorConfig[0].firingThresholdPercentage = 80;
    memcpy(&flywheelConfig.motorConfig[1], &flywheelConfig.motorConfig[0], sizeof(MotorConfig));
    memcpy(&flywheelConfig.motorConfig[2], &flywheelConfig.motorConfig[0], sizeof(MotorConfig));
    memcpy(&flywheelConfig.motorConfig[3], &flywheelConfig.motorConfig[0], sizeof(MotorConfig));

    ClosedLoopFlywheelParams clFlywheelParams;
    clFlywheelParams.idleRpm = 24000;
    clFlywheelParams.idleTime_ms = 500;
    clFlywheelParams.spindownSpeed = 9;

    PusherConfig pusherConfig;
    pusherConfig.type = PusherType_PUSHER_SOLENOID_CLOSEDLOOP;
    pusherConfig.controlInputs_count = 1;
    pusherConfig.controlInputs[0].type = PusherControlInputType_PUSHER_REARMOST;
    pusherConfig.controlInputs[0].dataPin = 27;
    pusherConfig.which_PusherTimings = 2; // Guess here, intending solenoid
    pusherConfig.PusherTimings.solenoidTiming.extendTime = 50; // Making up numbers here
    pusherConfig.PusherTimings.solenoidTiming.retractTime = 70;
    pusherConfig.pusherVoltage_mv = 11100; // 3S voltage for no particular reason
    pusherConfig.pusherDirectionReverse = false;

    strncpy(allFields.blasterName, defaultName, 20);
    allFields.has_controlconfig = true;
    memcpy(&allFields.controlconfig, &controlConfig, sizeof(ControlConfig));
    allFields.has_controlParams = true;
    memcpy(&allFields.controlParams, &controlParams, sizeof(ControlParams));
    allFields.has_flywheelConfig = true;
    memcpy(&allFields.flywheelConfig, &flywheelConfig, sizeof(FlywheelConfig));
    allFields.which_flywheelParams = 1; // I think this means closed loop, assuming 0 is none and 2 is open loop
    memcpy(&allFields.flywheelParams.closedLoopFlywheelParams, &clFlywheelParams, sizeof(ClosedLoopFlywheelParams));
    allFields.has_pusherConfig = true;
    memcpy(&allFields.pusherConfig, &pusherConfig, sizeof(PusherConfig));
    allFields.hardwareVersion = HardwareVersion_VERSION_0P4;

    return allFields;
}

void TestProtos(void)
{
    std::string buffer;
    Blaster test1 = Blaster_init_zero;

    strncpy(test1.blasterName, defaultName, 20);
    test1.has_controlconfig = true;
    test1.controlconfig.triggerSwitchPin = 15;

    ProtoEncodeToString(test1, &buffer);

    shell.printf("Proto Buffer is: %d|0x", buffer.size());
    for (int ndx = 0; ndx < buffer.size(); ndx++) {
        shell.printf(" %02x", buffer.c_str()[ndx]);
    }
    shell.printf("|\n");

    Blaster received = Blaster_init_zero;

    ProtoDecodeFromString(&received, buffer);
    shell.printf("Received blaster:\n");
    PrettyPrintBlaster(shell, received);

    assert(strncmp(test1.blasterName, received.blasterName, 20) == 0);

    shell.printf("\nAll Field blaster test:\n");
    uint8_t allFieldBuffer[1024]; // Much longer than it actually need to be

    Blaster allFields = getAllFieldsBlaster();
    PrettyPrintBlaster(shell, allFields);

    ProtoEncodeToString(allFields, &buffer);

    shell.printf("Proto Buffer is %d bytes long: |0x", buffer.size());
    for (int ndx = 0; ndx < buffer.size(); ndx++) {
        shell.printf("%02x", buffer.c_str()[ndx]);
    }
    shell.printf("|\n%s\n", buffer.c_str());
}

static void PrettyPrintHardwareVersion(SimpleSerialShell& printer, Blaster& blaster)
{
    printer.printf("Detlaff Board Version: %s\n",
        ((blaster.hardwareVersion == HardwareVersion_VERSION_0P1)          ? "0.1"
                : (blaster.hardwareVersion == HardwareVersion_VERSION_0P2) ? "0.2"
                : (blaster.hardwareVersion == HardwareVersion_VERSION_0P3) ? "0.3"
                : (blaster.hardwareVersion == HardwareVersion_VERSION_0P4) ? "0.4"
                                                                           : "Invalid"));
}

static void PrettyPrintControlConfig(SimpleSerialShell& printer, Blaster& blaster)
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

static void PrettyPrintControlParams(SimpleSerialShell& printer, Blaster& blaster)
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
            printer.printf("\t\tBurst Length %d, Burst Type: ", fireModeEntry.burstLength);
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
            printer.printf(" RPMs: |");
            for (int rpmNdx = 0; rpmNdx < fireModeEntry.rpm_count; rpmNdx++) {
                printer.printf(" %d |", fireModeEntry.rpm[rpmNdx]);
            }
            printer.printf("\n");
        }
    } else {
        printer.printf("No Control Params\n");
    }
}

static void PrettyPrintFlywheelConfig(SimpleSerialShell& printer, Blaster& blaster)
{
    if (blaster.has_flywheelConfig) {
        printer.printf("Flywheel Config\n");
        FlywheelConfig flywheelConfig = blaster.flywheelConfig;

        printer.printf("\tNumber of motors: %d\n", flywheelConfig.numMotors);

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

            printer.printf("\t\tMotor %d: KV: %d, Idle RPM %d, Firing Threshold %d%\n",
                ndx, motorConfig.motorKv, motorConfig.idleRPM, motorConfig.firingThresholdPercentage);
        }
    } else {
        printer.printf("\t\tNo Flywheel Config\n");
    }
}

static void PrettyPrintFlywheelParams(SimpleSerialShell& printer, Blaster& blaster)
{
    printer.printf("Flywheel Params of type: ");
    if (blaster.which_flywheelParams == 0) {
        printer.printf("Closed Loop\n");

        ClosedLoopFlywheelParams closedLoopFlywheelParams = blaster.flywheelParams.closedLoopFlywheelParams;

        printer.printf("\tIdle RPM: %d, Idle Time: %d ms, Spindown Speed: %d\n",
            closedLoopFlywheelParams.idleRpm,
            closedLoopFlywheelParams.idleTime_ms,
            closedLoopFlywheelParams.spindownSpeed);
    } else {
        printer.printf("Open Loop\n");

        OpenLoopFlywheelParams openLoopFlywheelParams = blaster.flywheelParams.openLoopFlywheelParams;

        printer.printf("\tIdle Duty Cycle: %d, Idle Time: %d ms, Minimum Rev Time: %d ms, Minimum Idle Rev Time: %d ms, Spindown Speed: %d\n",
            openLoopFlywheelParams.idleDutyCycle,
            openLoopFlywheelParams.idleTime_ms,
            openLoopFlywheelParams.minimumRevTime_ms,
            openLoopFlywheelParams.minimumIdleRevTime_ms,
            openLoopFlywheelParams.spindownSpeed);
    }
}

static void prettyPrintPusherConfig(SimpleSerialShell& printer, Blaster& blaster)
{
    int ndx;

    PusherConfig pusherConfig = blaster.pusherConfig;
    printer.printf("Pusher Params of type: %s\n",
        ((pusherConfig.type == PusherType_PUSHER_MOTOR_CLOSEDLOOP)             ? "Motor closed Loop"
                : (pusherConfig.type == PusherType_PUSHER_SOLENOID_CLOSEDLOOP) ? "Solenoid closed loop"
                : (pusherConfig.type == PusherType_PUSHER_MOTOR_OPENLOOP)      ? "Motor open loop"
                : (pusherConfig.type == PusherType_PUSHER_SOLENOID_OPENLOOP)   ? "Solenoid open loop"
                                                                               : "Invalid"));

    printer.printf("\tPusher control inputs:\n");

    for (ndx = 0; ndx < pusherConfig.controlInputs_count; ndx++) {
        printer.printf("\t\t%d: Pin: %d Type: %s\n", ndx, pusherConfig.controlInputs[ndx].dataPin,
            ((pusherConfig.controlInputs[ndx].type == PusherControlInputType_PUSHER_REARMOST)          ? "Rearmost switch"
                    : (pusherConfig.controlInputs[ndx].type == PusherControlInputType_PUSHER_FOREMOST) ? "Foremost swtich"
                    : (pusherConfig.controlInputs[ndx].type == PusherControlInputType_PUSHER_POSITION) ? "Pusher position"
                                                                                                       : "Invalid"));
    }
    if (ndx == 0) {
        printer.printf("\tNo pusher control inputs\n");
    }

    if (pusherConfig.which_PusherTimings == 0) {
        printer.printf("\tMotor stall timing: %d ms\n", pusherConfig.PusherTimings.motorTiming.stallTime_ms);
    } else {
        printer.printf("\tSolenoid extend time: %d ms, retract time: %d ms\n", pusherConfig.PusherTimings.solenoidTiming.extendTime,
            pusherConfig.PusherTimings.solenoidTiming.retractTime);
    }

    printer.printf("\tPusherVoltage: %d mv\n", pusherConfig.pusherVoltage_mv);
    printer.printf("\tPusher direction: %s\n", (pusherConfig.pusherDirectionReverse ? "Reverse" : "Forward"));
}

// No Pusher params because they live in the control protos
