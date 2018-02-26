#include "digging_queue.h"

namespace tfr_mining
{
    DiggingQueue::DiggingQueue() : sets{}
    {
        // Temporary test sets
        for (int j = 0; j < 5; j++) {
            DiggingSet set;
            for (int i = 0; i < 5; i++) {
                std::vector<double> state { (-3.14159265 / 2) + (j * (3.14159265/4)) , 0.5, 0.5 + (0.2 * i), 0.0 };
                set.insertState(state, 3.5);
            }
            sets.push(set);
        }
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
