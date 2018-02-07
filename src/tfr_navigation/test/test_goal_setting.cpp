#include <gtest/gtest.h>
#include <navigation_goal_manager.h>
#include <tfr_msgs/NavigationAction.h>
#include <tfr_utilities/location_codes.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <iostream>

//this is test code so sure whatev global defines are fine

class GoalManager : public ::testing::Test
{
    protected:

        virtual void SetUp()
        {      
        }

        virtual void TearDown()
        {
        }
        move_base_msgs::MoveBaseGoal nav_goal{};
        double SAFE_MINING_DISTANCE = 5.1;
        double FINISH_LINE = 0.84;

};

TEST_F(GoalManager, initializeMiningGoal)
{
    NavigationGoalManager::GeometryConstraints 
        constraints(SAFE_MINING_DISTANCE, 
                FINISH_LINE);
    NavigationGoalManager manager("bin_footprint",constraints);
    manager.initialize_goal(nav_goal, tfr_utilities::LocationCode::MINING);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.x , 
            SAFE_MINING_DISTANCE);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.y , 0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.z , 0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.x ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.y ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.z ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.w ,1);
}

TEST_F(GoalManager, initializeDumpingGoal)
{
    NavigationGoalManager::GeometryConstraints 
        constraints(SAFE_MINING_DISTANCE, 
                FINISH_LINE);
    NavigationGoalManager manager("bin_footprint",constraints);
    manager.initialize_goal(nav_goal, tfr_utilities::LocationCode::DUMPING);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.x , 
            FINISH_LINE);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.y , 0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.z , 0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.x ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.y ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.z ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.w ,1);
}

TEST_F(GoalManager, badConstraints)
{
    NavigationGoalManager::GeometryConstraints 
        constraints(-1, 
                FINISH_LINE);
    NavigationGoalManager manager("bin_footprint", constraints);
    std::cout << "Note: you should have seen a warning when running this" 
        <<" test case, this is defined behavior." << std::endl;
}

int main(int argc, char **argv)
{
    ros::Time::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
