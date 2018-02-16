/*
 * Shared codes and enums for teleop transmission
 * */
#ifndef TELEOP_CODE_H
#define TELEOP_CODE_H

#include <cstdint>

namespace tfr_utilities
{
    /*
     * Codes for commands send back and forth in teleop mode
     * */
    enum class TeleopCode: uint8_t
    {
        NONE,
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,
        DIG,
        DUMP,
        RESET
    };
}
#endif
