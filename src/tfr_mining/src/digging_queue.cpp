#include "digging_queue.h"

namespace tfr_mining
{
    // Must be a private node handle ("~")
    DiggingQueue::DiggingQueue(ros::NodeHandle nh) : sets{}
    {
        XmlRpc::XmlRpcValue positions;

        if (!nh.getParam("positions", positions)) {
            ROS_ERROR("Error loading positions, exiting");
            return;
        }

        for (int i = 0; i < positions.size(); i++) {
            DiggingSet toAdd;
            for (int j = 0; j < positions[i].size(); j++)
            {
                std::vector<double> state;
                for (int angle = 0; angle < 5; angle++) {
                    state.push_back(positions[i][j][angle]);
                }
                toAdd.insertState(state, 4.5); // Just use a constant time for simplicity.
            }
            sets.push(toAdd);
        }
    }

    bool DiggingQueue::isEmpty()
    {
        return sets.empty();
    }

    DiggingSet DiggingQueue::popDiggingSet()
    {
        DiggingSet set = sets.front();
        sets.pop();
        return set;
    }
}
