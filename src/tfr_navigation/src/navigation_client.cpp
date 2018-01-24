#include <navigation_client.h>
        
/**
 *  default constructor, brings up a server to listen to a specific action
 * */
NavigationClient::NavigationClient(std::string action_name):client{action_name, true}
{
    client.waitForServer();
}
       
/**
 * Signals the server to drop whatever it may be doing and navigate to
 * the mining zone.
 */
void NavigationClient::navigate_to_mining()
{
    //lets navigate to the mining zone!
    navigate(tfr_msgs::NavigationGoal::TO_MINING);
}

/**
 * Navigate to an arbirtrary defined place as defined in the Navigation
 * Goal constants header.
 *
 * action - The byte code corresponding to a location in the world map
 * */
void NavigationClient::navigate(uint8_t action)
{
    tfr_msgs::NavigationGoal goal;
    goal.location_code= action;
    client.sendGoal(goal,
            boost::bind(&NavigationClient::finished,this, _1,_2),
            Client::SimpleActiveCallback(),
            boost::bind(&NavigationClient::feedback,this, _1));
}

/**
 *  Standard feedback callback for action client.
 * */
void NavigationClient::feedback(const tfr_msgs::NavigationFeedbackConstPtr& feedback)
{
    ROS_INFO("feedback recieved, code: %d", feedback->status);
}

/**
 * Standard finishing callback for action client. 
 * */
void NavigationClient::finished(const actionlib::SimpleClientGoalState &state, const
        tfr_msgs::NavigationResultConstPtr &result)
{
    ROS_INFO("result recieved, code: %d", result->status);
    ros::shutdown();
}
