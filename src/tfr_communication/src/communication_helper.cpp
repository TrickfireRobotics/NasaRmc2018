#include "communication_helper.h"

namespace tfr_communication
{
    CommunicationHelper::CommunicationHelper()
    {
    }

    std::string CommunicationHelper::GetEcho()
    {
        return "Communication System Online";
    }
}