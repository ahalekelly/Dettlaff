#ifndef SOLENOID_H
#define SOLENOID_H

#include "types.h"

// clang-format off
/* For some strange reason SimpleSerialShell doesn't include Steam.h, so we have to include it before the first time it's included or the library breaks
 * This appears to be alphabetical by file and not by the parent files that invokes it, so just include stream.h everywhere to be safe
 */
#include <Stream.h>
#include <SimpleSerialShell.h>
// clang-format on

extern SimpleSerialShell& shell;

// The solenoid extend time global from main.cpp
// I don't like it, but this is a workaround for now
extern uint16_t solenoidExtendTime_ms;

int shellCommandSolenoid(int argc, char** argv);

#endif // SOLENOID_H