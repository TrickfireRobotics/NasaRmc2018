/*
 * Shared codes and for querying status from the control system
 * */
#ifndef CONTROL_CODE_H
#define CONTROL_CODE_H

#include <cstdint>

namespace tfr_utilities
{
    /*
     * Codes for commands send back and forth in teleop mode
     * */
    enum class BinCode: uint8_t
    {
        RAISED,
        LOWERED,
        IN_BETWEEN
    };
}
#endif
