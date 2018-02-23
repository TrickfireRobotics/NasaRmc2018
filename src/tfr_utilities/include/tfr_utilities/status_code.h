#ifndef STATUS_CODE_H
#define STATUS_CODE_H

#include <cstdint>
#include <string>

/**
 * Definitions of Sub Systems.
 *
 * These codes use the top 8 bits to uniquely identify the sub system a
 * particular status code belongs to and should never be changed.
 */
enum class SubSystem: uint16_t
{
    EXC = 0b0000000100000000,
    LOC = 0b0000001000000000,
    NAV = 0b0000010000000000,
    MIN = 0b0000100000000000,
    DMP = 0b0001000000000000
};

/**
 * Defines the list of all status codes used by all Sub Systems.
 *
 * These codes consist of two parts:
 *  - The top 8 bits which identify the sub system
 *  - The bottom 8 bits which identify the status of the sub system
 *
 * When adding a new status code for a sub system, use the system identifier
 * defined in the SubSystem enum class above and then increase the previous 8
 * bit code by one. For example, if adding a new status code for the Executive
 * sub system and the most recently added status code is 0b0000000100000000, the
 * new status code should be defined as 0b0000000100000001.
 *
 * Don't forget to also add a message for any newly added status codes in
 * status_code.cpp.
 */
enum class StatusCode : uint16_t
{
    //Executive Status Codes
    EXC_OK = 0b0000000100000000,

    //Localization Status Codes
    LOC_OK = 0b0000001000000000,

    //Navigation Status Codes
    NAV_OK = 0b0000010000000000,

    //Mining Status Codes
    MIN_OK = 0b0000100000000000,

    //Dumping Status Codes
    DMP_OK = 0b0001000000000000
};

/**
 * Get the status message from a status code and status data.
 */
std::string getStatusMessage(StatusCode code, float data);

#endif