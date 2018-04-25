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
        static const float ANGLE_TOLERANCE = 0.01;
        //NOTES must match up with constants in model
        static const float ARM_TURNTABLE_MAX = 6.28319;
        static const float ARM_TURNTABLE_MIN = 0.0;
        static const float ARM_LOWER_MAX = 1.65806;
        static const float ARM_LOWER_MIN = 0.10472;
        static const float ARM_UPPER_MAX = 2.30383;
        static const float ARM_UPPER_MIN = 1.06465;
        static const float ARM_SCOOP_MAX = 1.44862;
        static const float ARM_SCOOP_MIN = -1.46608;
        static const float BIN_MAX = 0.785398;
        static const float BIN_MIN = 0.0;
    }

}
#endif
