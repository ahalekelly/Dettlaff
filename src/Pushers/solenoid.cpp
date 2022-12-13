#include <Pushers/solenoid.h>

/**************************************************************/
/******************* Shell Command Solenoid *******************/
/**************************************************************/

enum cCommandPositions
{
    cCommand,
    cFunction,
    cArg,
};

static constexpr size_t cMaxArgLen = strlen("getExtendTime");

int shellCommandSolenoid(int argc, char **argv)
{
    int ret = 0;

    // The must be enough arguments to at least call a function (for argument-less functions like Extend and Retract)
    assert(argc >= (cFunction + 1));

    if (strncmp(argv[cFunction], "getExtendTime", cMaxArgLen) == 0)
    {
        shell.printf("The Solenoid extension time is %u\n", solenoidExtendTime_ms);
    }
    else if (strncmp(argv[cFunction], "help", cMaxArgLen) == 0)
    {
        shell.printf("getExtendTime");
    }

    return ret;
}
