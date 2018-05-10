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
        STOP_DRIVEBASE = 0,
        FORWARD = 1,
        BACKWARD = 2,
        LEFT = 3,
        RIGHT = 4,
        CLOCKWISE = 5,
        COUNTERCLOCKWISE = 6,
        DIG = 7,
        DUMP = 8,
        RESET_DUMPING = 9,
        RESET_STARTING = 10, 
        DRIVING_POSITION= 11,
        RAISE_ARM = 12,

    };
}
#endif
