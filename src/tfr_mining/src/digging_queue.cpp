#include "digging_queue.h"

namespace tfr_mining
{
    DiggingQueue::DiggingQueue() : sets{}
    {
        // TODO: Populate sets
        // Temporary test set
        DiggingSet set;
        for (int i = 0; i < 5; i++) {
            std::vector<double> state {};
            set.insertState(state, 2.0);
        }
        sets.push(set);
    }

    bool DiggingQueue::isEmpty()
    {
        return sets.empty();
    }

    DiggingSet DiggingQueue::popDiggingSet() {
        DiggingSet set = sets.front();
        sets.pop();
        return set;
    }
}