#include "digging_set.h"

namespace tfr_mining
{
    DiggingSet::DiggingSet() : states{}, time_estimate{0}
    {

    }

    void DiggingSet::insertState(std::vector<double> state, double time)
    {
        states.push(state);
        time_estimate += time;
    }

    bool DiggingSet::isEmpty()
    {
        return states.empty();
    }

    std::vector<double> DiggingSet::popState()
    {
        std::vector<double> state = states.front();
        states.pop();
        return state;
    }

    double DiggingSet::getTimeEstimate()
    {
        return time_estimate;
    }
}
