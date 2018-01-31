#include <navigation_client.h>
        
/**
 *  default constructor, brings up a server to listen to a specific action
 * */
NavigationClient::NavigationClient(std::string action_name):client{action_name, true}
{
    client.waitForServer();
}


/**
 * Nonblocking call to Navigate to an arbirtrary defined place as defined in the Navigation
 * Goal constants header.
 *
 * action - The byte code corresponding to a location in the world map
 * */
void NavigationClient::navigate(tfr_msgs::LocationCode location)
{
    tfr_msgs::NavigationGoal goal;
    //messages can't support user defined types
    goal.location_code= static_cast<uint8_t>(location);


    /*Maintenece Note:
     *  The SimpleActiveCallback() method below doesn't do anything, it's a
     *  placeholder provided by the library maintainers to pass into the method
     *  if you don't have a relevant callback to pass in.
     * */
    client.sendGoal(goal,
            boost::bind(&NavigationClient::finished,this, _1,_2),
            Client::SimpleActiveCallback(),
            boost::bind(&NavigationClient::feedback,this, _1));
}

/**
 * Stops all goals on the navigation action server
 * */
void NavigationClient::stop_all()
{
    client.cancelAllGoals();
}

/**
 * Nonblocking gets the state of the action server
 * */
actionlib::SimpleClientGoalState NavigationClient::get_state()
{
    return client.getState();
}




/**
 *  Standard feedback callback for action client.
 * */
void NavigationClient::feedback(const tfr_msgs::NavigationFeedbackConstPtr& feedback)
{
    ROS_INFO("feedback recieved");
}

/**
 * Standard finishing callback for action client. 
 * */
void NavigationClient::finished(const actionlib::SimpleClientGoalState &state, const
        tfr_msgs::NavigationResultConstPtr &result)
{
    ROS_INFO("result recieved");
}
