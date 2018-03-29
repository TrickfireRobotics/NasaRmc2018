/****************************************************************************************
 * File:            digging_set.h
 * 
 * Purpose:         This class implements a set of digging states with a time
 *                  estimate. It is intended to be instantiated only by
 *                  DiggingQueue, and then used by the DiggingActionServer after
 *                  being returned out.
 ***************************************************************************************/
#ifndef DIGGING_SET_H
#define DIGGING_SET_H

#include <queue>
#include <vector>

namespace tfr_mining
{
    class DiggingSet
    {
    public:
        DiggingSet();
        ~DiggingSet() = default;
        /**
         * Inserts a new state into this set, and increases the time estimate
         * accordingly.
         **/
        void insertState(std::vector<double> state, double time);

        /**
         * Returns whether this set is empty or not.
         **/
        bool isEmpty();

        /**
         * Returns the next state in this set and removes it.
         **/
        std::vector<double> popState();

        /**
         * Gets the time estimate for the set as a whole (cumulative for all
         * states).
         **/
        double getTimeEstimate();
    private:
        std::queue<std::vector<double> > states;
        double time_estimate;
    };
}

#endif // DIGGING_SET_H