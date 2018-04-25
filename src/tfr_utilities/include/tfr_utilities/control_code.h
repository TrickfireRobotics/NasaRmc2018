/*
 * Shared codes and for querying status from the control system
 * */
#ifndef CONTROL_CODE_H
#define CONTROL_CODE_H

#include <cstdint>

namespace tfr_utilities
{
    namespace JointAngle
    {
        //NOTES must match up with constants in model
        static const float ARM_TURNTABLE_MAX = 6.28319;
        static const float ARM_TURNTABLE_MIN = 0.0;
        static const float ARM_LOWER_MAX = 1.52;
        static const float ARM_LOWER_MIN = 0.06;
        static const float ARM_UPPER_MAX = 2.2;
        static const float ARM_UPPER_MIN = 0.99;
        static const float ARM_SCOOP_MAX = 1.1;
        static const float ARM_SCOOP_MIN = -1.6;
        static const float BIN_MAX = 0.785398;
        static const float BIN_MIN = 0.0;
    }

}
#endif
