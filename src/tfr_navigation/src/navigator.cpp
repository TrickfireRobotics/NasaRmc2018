#include <navigator.h>

/**
 *  constructs the sever and binds it to it's execution callback
 *  displays set parameters and warnings to the user
 * */
Navigator::Navigator(ros::NodeHandle &n,
        const NavigationGoalManager::GeometryConstraints &constraints, 
        std::string name) : node{n}, 
        goal_manager(constraints),
        server{n, name, boost::bind(&Navigator::navigate, this, _1) ,false}, 
        nav_stack{"move_base", true},
        navigation{navigation_label, true},
        action_name{name}
{
    ROS_DEBUG("Navigation server constructed %f", ros::Time::now().toSec());
    //get parameters
    ros::param::param<std::string>("~odometry_topic", odometry_topic,
            "/fused_odom");
    ros::param::param<float>("~rate", rate, 1);
    ros::param::param<std::string>("~frame_id", frame_id, "base_footprint");

    odom_subscriber = node.subscribe(odometry_topic, 5,
            &Navigator::update_position, this);

    //display parameters to the user
    ROS_DEBUG(" name:           %s", action_name.c_str());
    ROS_DEBUG(" frame_id:       %s", frame_id.c_str());
    ROS_DEBUG(" odometry_topic: %s", odometry_topic.c_str());
    ROS_DEBUG(" rate:           %f", rate);

    ROS_INFO("Navigation server connecting to nav_stack");
    nav_stack.waitForServer();
    ROS_INFO("Navigation server connected to nav_stack");
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
 *  NOTE careful with the shared pointers in this class if threading becomes
 *  priority.
 * */
void Navigator::navigate(const tfr_msgs::NavigationGoalConstPtr &goal)
{
    ROS_INFO("Navigation server started");

    //start with initial goal
    goal_manager.location_code = goal->location_code;
    nav_goal = goal_manager.initialize_goal();
    nav_stack.sendGoal(goal);

    ros::Rate r(rate);  
    r.sleep(); //this pause is for debugging only DELETE

    //test for completion
    while (nav_stack.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        //Deal with preemption or error
        if (server.isPreemptRequested() || !ros::ok()) 
        {
            ROS_INFO("%s: preempted", action_name.c_str());
            nav_stack.cancelAllGoals();
            update_result();
            server.setPreempted(result);
            success = false;
            return;
        }
        //main case, update nav goal
        else
        {
            if (goal_manager.location_code == tfr_msgs::NavigationGoal::TO_MINING)
            {
                nav_goal = goal_manager.get_updated_mining_goal(current_position.pose.pose);
                nav_stack.sendGoal(goal);
            }

            update_feedback();
            server.publishFeedback(feedback);
            ROS_INFO("servicing goal, %f", feedback.header.stamp.toSec());
            r.sleep();
        }
    }

    update_result();
    if (nav_stack.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        server.setSucceeded(result);
    }
    else (false)
    {
        nav_stack.cancelAllGoals();
        server.setAborted(result);
    }
    ROS_INFO("Navigation server finished");
}


/*
 * Prepare a feedback message for sending
 * */
void Navigatore::update_result()
{
    result.header.stamp = ros::Time::now();
    result.header.frame_id = frame_id;
    result.current = current_position.pose.pose;
    result.goal = nav_goal.target_pose.pose;
}

/*
 * Prepare a feedback message for sending
 * */
void Navigatore::update_feedback()
{
    feedback.header.stamp = ros::Time::now();
    feedback.header.frame_id = frame_id;
    feedback.current = current_position.pose.pose;
    feedback.goal = nav_goal.target_pose.pose;
}


/**
 * Callback for updating the most recent position
 * */
void Navigator::update_position(const nav_msgs::OdometryConstPtr &msg)
{
    //get the pose without the covariance, not needed
    //need to use shared pointer here for thread safety see:
    //https://answers.ros.org/question/53234/processing-an-image-outside-the-callback-function/
    current_position = *msg;
}
