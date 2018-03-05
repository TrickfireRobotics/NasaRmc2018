#include "digging_queue.h"

namespace tfr_mining
{
    // Must be a private node handle ("~")
    DiggingQueue::DiggingQueue(ros::NodeHandle nh) : sets{}
    {
        // Make 4 digging sets, going from 45 degrees right to 90 degrees left
        //  - The reasoning for this is so that we can always dump our excess
        //    to the left. We'll fill in our previous holes a bit as we go, and
        //    it ensures that we never put digging material in a place we'll dig
        //    later
        for (int j = 0; j < 4; j++)
        {
            DiggingSet set;
            generateDigAndDump(nh, set, 3.14159265 / 4 - ((3.14159265 / 4) * j), 1);
            generateDigAndDump(nh, set, 3.14159265 / 4 - ((3.14159265 / 4) * j), 2);
            generateDigAndDump(nh, set, 3.14159265 / 4 - ((3.14159265 / 4) * j), 3);
            generateDigAndDump(nh, set, 3.14159265 / 4 - ((3.14159265 / 4) * j), 4);
            sets.push(set);
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

    void DiggingQueue::generateDigAndDump(ros::NodeHandle &nh, DiggingSet &set, double rotation, int dig_number)
    {
        // Ready (constant no matter what)
        std::vector<double> ready_pos;
        double ready_time;
        if (!nh.getParam("positions/ready", ready_pos) || !nh.getParam("positions/ready_time", ready_time)) {
            throw std::runtime_error("Error loading ready state");
        }
        std::vector<double>::iterator it = ready_pos.begin();
        ready_pos.insert(it, rotation);
        set.insertState(ready_pos, ready_time);

        // Dig position
        std::vector<double> dig_pos;
        double dig_time;
        std::stringstream dig_str, dig_time_str;
        dig_str << "positions/dig" << dig_number;
        dig_time_str << dig_str.str() << "_time";
        if (!nh.getParam(dig_str.str(), dig_pos) || !nh.getParam(dig_time_str.str(), dig_time)) {
            throw std::runtime_error("Error loading " + dig_str.str());
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
        if (!nh.getParam(scoop_str.str(), scoop_pos) || !nh.getParam(scoop_time_str.str(), scoop_time)) {
            throw std::runtime_error("Error loading " + scoop_str.str());
        }
        it = scoop_pos.begin();
        scoop_pos.insert(it, rotation);
        set.insertState(scoop_pos, scoop_time);

        // Out (constant no matter what)
        std::vector<double> out_pos;
        double out_time;
        if (!nh.getParam("positions/out", out_pos) || !nh.getParam("positions/out_time", out_time)) {
            throw std::runtime_error("Error loading out state");
        }
        it = out_pos.begin();
        out_pos.insert(it, rotation);
        set.insertState(out_pos, out_time);

        if (dig_number > 2) {
            // Dump into the bin
            std::vector<double> dump_pos;
            double dump_time;
            if (!nh.getParam("positions/bindump", dump_pos) || !nh.getParam("positions/bindump_time", dump_time)) {
                throw std::runtime_error("Error loading bindump state");
            }
            set.insertState(dump_pos, dump_time);

            std::vector<double> release_pos;
            double release_time;
            if (!nh.getParam("positions/binrelease", release_pos) || !nh.getParam("positions/binrelease_time", release_time)) {
                throw std::runtime_error("Error loading binrelease state");
            }
            set.insertState(release_pos, release_time);
        } else {
            // We need to dump to the side first
            std::vector<double> dump_pos;
            double dump_time;
            if (!nh.getParam("positions/excessdump", dump_pos) || !nh.getParam("positions/excessdump_time", dump_time)) {
                throw std::runtime_error("Error loading excessdump state");
            }
            it = dump_pos.begin();
            dump_pos.insert(it, rotation + (3.14159265 / 4));
            set.insertState(dump_pos, dump_time);

            std::vector<double> release_pos;
            double release_time;
            if (!nh.getParam("positions/excessrelease", release_pos) || !nh.getParam("positions/excessrelease_time", release_time)) {
                throw std::runtime_error("Error loading excessrelease state");
            }
            it = release_pos.begin();
            release_pos.insert(it, rotation + (3.14159265 / 4));
            set.insertState(release_pos, release_time);
        }
    }
}
