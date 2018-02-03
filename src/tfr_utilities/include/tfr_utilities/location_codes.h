#ifndef LOCATION_CODES_H
#define LOCATION_CODES_H

#include<cstdint>
namespace tfr_utilities
{
    enum class LocationCode : uint8_t {
        UNSET,
        MINING,
        DUMPING
    };
}

#endif
