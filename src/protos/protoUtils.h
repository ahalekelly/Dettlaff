#pragma once

#include <pb_decode.h>
#include <pb_encode.h>

#include "blaster.pb.h"

// Enables shell commands
// clang-format off
/* For some strange reason SimpleSerialShell doesn't include Steam.h, so we have to include it before the first time it's included or the library breaks
 * This appears to be alphabetical by file and not by the parent files that invokes it, so just include stream.h everywhere to be safe
 */
#include <string>
#include <Stream.h>
#include <SimpleSerialShell.h>
// clang-format on
extern SimpleSerialShell& shell;

void TestProtos(void);

int ProtoDecodeFromString(Blaster* received, const std::string& input);
int ProtoEncodeToString(const Blaster& received, std::string* output);

void PrettyPrintBlaster(SimpleSerialShell& printer, Blaster& blaster);
