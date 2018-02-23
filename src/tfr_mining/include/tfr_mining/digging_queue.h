/****************************************************************************************
 * File:            digging_queue.h
 * 
 * Purpose:         This class implements a queue of digging sets, used to
 *                  execute large sets of digging actions in a row. It
 *                  instantiates all of its stored states on construction, and
 *                  we then pop from it until we run out of states or we
 *                  run out of time to dig.
 ***************************************************************************************/
#ifndef DIGGING_QUEUE_H
#define DIGGING_QUEUE_H

#include "digging_set.h"
#include <queue>

namespace tfr_mining
{
    class DiggingQueue
    {
    public:
        /**
         * Constructs the queue and instantiates all of the digging sets inside
         * it.
         **/
        DiggingQueue();
        ~DiggingQueue() = default;

        /**
         * Returns whether this queue is empty or not.
         **/
        bool isEmpty();

        /**
         * Returns the next set in the queue and removes it.
         **/
        DiggingSet popDiggingSet();
    private:
        std::queue<DiggingSet> sets;
    };
}

#endif // DIGGING_QUEUE_H