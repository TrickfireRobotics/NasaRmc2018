#include <navigator.h>

/**
 *  constructs the sever and binds it to it's execution callback
 *  displays set parameters and warnings to the user
 * */
Navigator::Navigator(ros::NodeHandle &n,std::string name) : node{n},  server{n, name,
    boost::bind(&Navigator::navigate, this, _1) ,false}, action_name{name},
    goal_manager()
{
    ROS_INFO("Navigation server constructed");
    //get parameters
    node.param<std::string>("~odometry_topic", odometry_topic,
            "/fused_odom");
    node.param<float>("~rate", rate, 1);
    node.param<std::string>("~frame_id", frame_id, "base_footprint");

    odom_subscriber = node.subscribe(odometry_topic, 5,
            &Navigator::update_position, this);

    //display parameters to the user
    ROS_DEBUG(" name:           %s", action_name.c_str());
    ROS_DEBUG(" frame_id:       %s", frame_id.c_str());
    ROS_DEBUG(" odometry_topic: %s", odometry_topic.c_str());
    ROS_DEBUG(" rate:           %f", rate);


    server.start();

    ROS_INFO("Navigation server awaiting connection");
}

/**
 *  Goal: 
 *      -uint8_t code corresponding to where we want to navigate. Goal list is
 *      described in Navigation.action in the tfr_msgs package
 *  Feedback:
 *      -uint8_t code corresponding to our current status described in 
 *      Navigation.action in the tfr_msgs package
 *      -Pose describing the current position
 *      -Pose describing the final targeted position
 *  Response
 *      -uint8_t code corresponding to the finsal status described in 
 *      Navigation.action in the tfr_msgs package
 *      -Pose describing our final position
 * */
void Navigator::navigate(const tfr_msgs::NavigationGoalConstPtr &goal)
{
    ROS_INFO("Navigation server started");

    //start with initial goal
    goal_manager.location_code = (*goal).location_code;
    goal_manager.initialize_goal();
    //TODO send goal to navigation stack 

    ros::Rate r(rate);  

    //test for completion
    //TODO hook up to navigation stack here

    while (true)
    {
        //Deal with preemption or error
        if (server.isPreemptRequested() || !ros::ok()) 
        {
            //TODO hook up to navigation stack here
            //cascade the preemption and wait
            ROS_INFO("%s: preempted", action_name.c_str());
            server.setPreempted();
            break;
        }
        //main case, update nav goal
        else
        {
            if (goal_manager.location_code == tfr_msgs::NavigationGoal::TO_MINING)
            {
                //grab local copy of shared ptr for safe shared memory
                auto position = current_position;
                goal_manager.update_mining_goal((*position).pose.pose);
                //TODO send updated goal to navigation stack
            }

            //TODO hookup status query to navigation stack here

            feedback.header.stamp = ros::Time::now();
            feedback.header.frame_id = frame_id;
            feedback.status = tfr_msgs::NavigationFeedback::OK;
            auto position = current_position;
            feedback.current = (*position).pose.pose;
            feedback.goal = goal_manager.nav_goal.target_pose.pose;
            server.publishFeedback(feedback);
            r.sleep();
        }
    }

    //TODO hook up to navigation stack here
    //test for success
    if (true)
    {
        result.header.stamp = ros::Time::now();
        result.header.frame_id = frame_id;
        result.status = tfr_msgs::NavigationResult::OK;
        auto position = current_position;
        result.current = (*position).pose.pose;
        result.goal = goal_manager.nav_goal.target_pose.pose;
        server.setSucceeded(result);
    }
    else 
    {
        result.status = tfr_msgs::NavigationResult::OK;
        server.setAborted(result);
    }
    ROS_INFO("Navigation server finished");
}


/**
 * Callback for updating the most recent position
 * */
void Navigator::update_position(const nav_msgs::OdometryConstPtr &msg)
{
    //get the pose without the covariance, not needed
    //need to use shared pointer here for thread safety see:
    //https://answers.ros.org/question/53234/processing-an-image-outside-the-callback-function/
    current_position = msg;
}
