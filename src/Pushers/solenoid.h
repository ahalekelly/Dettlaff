#ifndef SOLENOID_H
#define SOLENOID_H

#include "types.h"
#include "SimpleSerialShell.h"

extern SimpleSerialShell &shell;

// The solenoid extend time global from main.cpp
// I don't like it, but this is a workaround for now
extern uint16_t solenoidExtendTime_ms;

int shellCommandSolenoid(int argc, char **argv);

#endif // SOLENOID_H