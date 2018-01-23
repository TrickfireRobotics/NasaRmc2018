/**
 *  Test Class for the navigation action server, eventuall will be enveloped
 *  into the executive node. 
 *
 *  Attaches to the node named "navigation_action_server" in the global
 *  namespace.
 *
 *  Subscribes to all of the usual topics needed by a simple action client
 *
 * */
#include <ros/ros.h>
#include <ros/console.h>
#include <cstdint>
#include <tfr_msgs/NavigationAction.h>
#include <actionlib/client/simple_action_client.h>


class NavigationClient
{
    public:
        NavigationClient(std::string action_name):client{action_name, true}
        {
            client.waitForServer();
        }

        ~NavigationClient() {} 
        //delete generated methods
        NavigationClient(const NavigationClient&) = delete;
        NavigationClient& operator=(const NavigationClient&) = delete;
        NavigationClient(NavigationClient&&) = delete;
        NavigationClient& operator=(NavigationClient&&) = delete;
        
        /**
         * Signals the server to drop whatever it may be doing and navigate to
         * the mining zone.
         */
        void navigate_to_mining()
        {
            //lets navigate to the mining zone!
            navigate(tfr_msgs::NavigationGoal::TO_MINING);
        }

    private:
        /**
         * Navigate to an arbirtrary defined place as defined in the Navigation
         * Goal constants header.
         *
         * action - The byte code corresponding to a location in the world map
         * */
        void navigate(uint8_t action)
        {
            tfr_msgs::NavigationGoal goal;
            goal.cmd_code = action;
            client.sendGoal(goal,
                    boost::bind(&NavigationClient::finished,this, _1,_2),
                    Client::SimpleActiveCallback(),
                    boost::bind(&NavigationClient::feedback,this, _1));
        }


        /**
         *  Standard feedback callback for action client.
         * */
        void feedback(const tfr_msgs::NavigationFeedbackConstPtr& feedback)
        {
            ROS_INFO("feedback recieved, code: %d", feedback->status);
        }

        /**
         * Standard finishing callback for action client. 
         * */
        void finished(const actionlib::SimpleClientGoalState &state, const
                tfr_msgs::NavigationResultConstPtr &result)
        {
            ROS_INFO("result recieved, code: %d", result->status);
            ros::shutdown();
        }
        using Client = actionlib::SimpleActionClient<tfr_msgs::NavigationAction>; 
        

        //the action client
        Client client;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_action_client");
    ros::NodeHandle n;
    NavigationClient client("navigation_action_server");
    client.navigate_to_mining();

    ros::spin();
    return 0;
}
