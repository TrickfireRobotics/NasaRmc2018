#include "digging_queue.h"

namespace tfr_mining
{
    // Must be a private node handle ("~")
    DiggingQueue::DiggingQueue(ros::NodeHandle nh) : sets{}
    {
        // Uncomment this for the full digging operation (all 3 holes)
        /*for (int dig_pos = -1; dig_pos <= 1; dig_pos++) {
            DiggingSet set;
            // 3 digs, one straight forward and two 30 degrees to either side
            double angle = -dig_pos * 3.14159265 / 6;
            for (int dig = 1; dig <= 4; dig++) {
                generateDigAndDump(nh, set, angle, dig);
            }
            sets.push(set);
        }*/

        // Uncomment this for a single full digging operation (1 hole, 4 digs, with )
        /*DiggingSet set;
        for (int dig = 1; dig <= 4; dig++) {
            generateDigAndDump(nh, set, 0.0, dig);
        }
        sets.push(set);*/

        // Do a single surface-level dig
        //  - To change the digging depth, change the last parameter to anything from 1-4.
        //    Do note that this uses different positions from digging_queue_templates.yaml
        DiggingSet set;
        generateSingleDig(nh, set, 0.0, 1);
        sets.push(set);
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

    void DiggingQueue::generateSingleDig(ros::NodeHandle &nh, DiggingSet &set, double rotation, int dig_number) {
        // Ready (constant no matter what)
        std::vector<double> ready_pos;
        double ready_time;
        if (!nh.getParam("positions/ready", ready_pos) || !nh.getParam("positions/ready_time", ready_time))
        {
            ROS_WARN("Error loading ready state");
            return;
        }
        auto it = ready_pos.begin();
        ready_pos.insert(it, rotation);
        set.insertState(ready_pos, ready_time);

        // Dig position
        std::vector<double> dig_pos;
        double dig_time;
        std::stringstream dig_str, dig_time_str;
        dig_str << "positions/dig" << dig_number;
        dig_time_str << dig_str.str() << "_time";
        if (!nh.getParam(dig_str.str(), dig_pos) || !nh.getParam(dig_time_str.str(), dig_time))
        {
            ROS_WARN_STREAM("Error loading " << dig_str.str());
            return;
        }
        it = dig_pos.begin();
        dig_pos.insert(it, rotation);
        set.insertState(dig_pos, dig_time);

        // Scoop position
        std::vector<double> scoop_pos;
        double scoop_time;
        std::stringstream scoop_str, scoop_time_str;
        scoop_str << "positions/scoop" << dig_number;
        scoop_time_str << scoop_str.str() << "_time";
        // Loads the scoop position and scoop estimated time from the parameter server (this code repeats
        // throughout all of the following code)
        if (!nh.getParam(scoop_str.str(), scoop_pos) || !nh.getParam(scoop_time_str.str(), scoop_time))
        {
            ROS_WARN_STREAM("Error loading " << scoop_str.str());
            return;
        }
        // Prepends the turntable position to the position vector, if necessary (this code repeats throughout
        // much of the following code, but not all of it)
        //  - This is so that, for some of the positions which are "relative" (like the digs), we specify
        //    which direcion it does it in, but for "static" positions (like dumping into the bin) we
        //    specify all four values in the parameter server.
        it = scoop_pos.begin();
        scoop_pos.insert(it, rotation);
        set.insertState(scoop_pos, scoop_time);

        // Out (constant no matter what)
        std::vector<double> out_pos;
        double out_time;
        if (!nh.getParam("positions/out", out_pos) || !nh.getParam("positions/out_time", out_time))
        {
            ROS_WARN("Error loading out state");
            return;
        }
        it = out_pos.begin();
        out_pos.insert(it, rotation);
        set.insertState(out_pos, out_time);
    }

    void DiggingQueue::generateDigAndDump(ros::NodeHandle &nh, DiggingSet &set, double rotation, int dig_number)
    {
        // Ready (constant no matter what)
        std::vector<double> ready_pos;
        double ready_time;
        if (!nh.getParam("positions/ready", ready_pos) || !nh.getParam("positions/ready_time", ready_time))
        {
            ROS_WARN("Error loading ready state");
            return;
        }
        auto it = ready_pos.begin();
        ready_pos.insert(it, rotation);
        set.insertState(ready_pos, ready_time);

        // Dig position
        std::vector<double> dig_pos;
        double dig_time;
        std::stringstream dig_str, dig_time_str;
        dig_str << "positions/dig" << dig_number;
        dig_time_str << dig_str.str() << "_time";
        if (!nh.getParam(dig_str.str(), dig_pos) || !nh.getParam(dig_time_str.str(), dig_time))
        {
            ROS_WARN_STREAM("Error loading " << dig_str.str());
            return;
        }
        it = dig_pos.begin();
        dig_pos.insert(it, rotation);
        set.insertState(dig_pos, dig_time);

        // Scoop position
        std::vector<double> scoop_pos;
        double scoop_time;
        std::stringstream scoop_str, scoop_time_str;
        scoop_str << "positions/scoop" << dig_number;
        scoop_time_str << scoop_str.str() << "_time";
        // Loads the scoop position and scoop estimated time from the parameter server (this code repeats
        // throughout all of the following code)
        if (!nh.getParam(scoop_str.str(), scoop_pos) || !nh.getParam(scoop_time_str.str(), scoop_time))
        {
            ROS_WARN_STREAM("Error loading " << scoop_str.str());
            return;
        }
        // Prepends the turntable position to the position vector, if necessary (this code repeats throughout
        // much of the following code, but not all of it)
        //  - This is so that, for some of the positions which are "relative" (like the digs), we specify
        //    which direcion it does it in, but for "static" positions (like dumping into the bin) we
        //    specify all four values in the parameter server.
        it = scoop_pos.begin();
        scoop_pos.insert(it, rotation);
        set.insertState(scoop_pos, scoop_time);

        // Out (constant no matter what)
        std::vector<double> out_pos;
        double out_time;
        if (!nh.getParam("positions/out", out_pos) || !nh.getParam("positions/out_time", out_time))
        {
            ROS_WARN("Error loading out state");
            return;
        }
        it = out_pos.begin();
        out_pos.insert(it, rotation);
        set.insertState(out_pos, out_time);

        if (dig_number > 2)
        {
            // Dump into the bin
            std::vector<double> dump_pos;
            double dump_time;
            if (!nh.getParam("positions/bindump", dump_pos) || !nh.getParam("positions/bindump_time", dump_time))
            {
                ROS_WARN("Error loading bindump state");
                return;
            }
            set.insertState(dump_pos, dump_time);

            std::vector<double> release_pos;
            double release_time;
            if (!nh.getParam("positions/binrelease", release_pos) || !nh.getParam("positions/binrelease_time", release_time))
            {
                ROS_WARN("Error loading binrelease state");
                return;
            }
            set.insertState(release_pos, release_time);
        } else {
            // We need to dump to the side first
            std::vector<double> dump_pos;
            double dump_time;
            if (!nh.getParam("positions/excessdump", dump_pos) || !nh.getParam("positions/excessdump_time", dump_time))
            {
                ROS_WARN("Error loading excessdump state");
                return;
            }
            it = dump_pos.begin();
            // Dumping 45 degrees to the left
            dump_pos.insert(it, rotation + (3.14159265 / 4));
            set.insertState(dump_pos, dump_time);

            std::vector<double> release_pos;
            double release_time;
            if (!nh.getParam("positions/excessrelease", release_pos) || !nh.getParam("positions/excessrelease_time", release_time))
            {
                ROS_WARN("Error loading excessrelease state");
                return;
            }
            it = release_pos.begin();
            release_pos.insert(it, rotation + (3.14159265 / 4));
            set.insertState(release_pos, release_time);
        }
    }
}
