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
        STOP_DRIVEBASE,
        STOP_TURNTABLE,
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,
        CLOCKWISE,
        COUNTERCLOCKWISE,
        DIG,
        DUMP,
        RESET_DUMPING,
        RESET_STARTING
    };
}
#endif
