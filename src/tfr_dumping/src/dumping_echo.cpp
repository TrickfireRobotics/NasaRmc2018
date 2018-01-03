/****************************************************************************************
 * This is the theoreticaly primary node for the dumping package.  Currently, all it does
 * is advertise its existence on the "dumping_echo" topic. 
 ***************************************************************************************/

#include "ros/ros.h"
#include "tfr_msgs/Echo.h"
#include "tfr_utilities/placeholder_utility.h"
#include "dumping_helper.h"

#include <sstream>


int main(int argc, char **argv)
{
    // "dumping_echo is the name of this node at run-time"
    ros::init(argc, argv, "dumping_echo");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
    ros::Publisher chatter_pub = n.advertise<tfr_msgs::Echo>("dumping_echo", 1000);

    ros::Rate loop_rate(10);

    // Placeholder unit-testable helper class
    tfr_dumping::DumpingHelper helper;

    // Placeholder utility function
    tfr_utilities::Foo();

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        tfr_msgs::Echo msg;

        std::stringstream ss;
        ss << helper.GetEcho();
        msg.message = ss.str();

        //ROS_INFO("%s", msg.message.c_str());

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
