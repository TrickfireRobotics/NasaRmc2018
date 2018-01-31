#include <gtest/gtest.h>
#include <navigation_goal_manager.h>
#include <tfr_msgs/NavigationAction.h>
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
        double SAFE_MINING_DISTANCE = 5.1;
        double MINING_LINE_LENGTH = 2.03;
        double FINISH_LINE = 0.84;

};
TEST_F(GoalManager, initializeMiningGoal)
{
    NavigationGoalManager::GeometryConstraints 
        constraints(SAFE_MINING_DISTANCE, 
                MINING_LINE_LENGTH, 
                FINISH_LINE);
    NavigationGoalManager manager(constraints, tfr_msgs::NavigationGoal::TO_MINING);
    auto nav_goal = manager.initialize_goal();
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.x , 
            SAFE_MINING_DISTANCE);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.y , 0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.z , 0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.x ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.y ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.z ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.w ,0);
}

TEST_F(GoalManager, initializeDumpingGoal)
{
    NavigationGoalManager::GeometryConstraints 
        constraints(SAFE_MINING_DISTANCE, 
                MINING_LINE_LENGTH, 
                FINISH_LINE);
    NavigationGoalManager manager(constraints,
            tfr_msgs::NavigationGoal::TO_DUMPING);
    auto nav_goal = manager.initialize_goal();
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.x , 
            FINISH_LINE);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.y , 0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.z , 0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.x ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.y ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.z ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.w ,0);
}

TEST_F(GoalManager, badConstraints)
{
    NavigationGoalManager::GeometryConstraints 
        constraints(-1, 
                MINING_LINE_LENGTH, 
                FINISH_LINE);
    NavigationGoalManager manager(constraints, tfr_msgs::NavigationGoal::TO_MINING);
    std::cout << "Note: you should have seen a warning when running this" 
        <<" test case, this is defined behavior." << std::endl;
}



TEST_F(GoalManager, adjustMiningGoalNeg)
{
    NavigationGoalManager::GeometryConstraints 
        constraints(SAFE_MINING_DISTANCE, 
                MINING_LINE_LENGTH, 
                FINISH_LINE);
    NavigationGoalManager manager(constraints, tfr_msgs::NavigationGoal::TO_MINING);
    geometry_msgs::Pose p{};
    p.position.x = 3.4;
    p.position.y = -.05;
    p.position.z = 0.4;
    p.orientation.x = .1;
    p.orientation.y = -.3;
    p.orientation.z = -.5;
    p.orientation.w = 1;
    auto nav_goal = manager.get_updated_mining_goal(p);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.x , 
            SAFE_MINING_DISTANCE);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.y , -.05);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.z , 0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.x ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.y ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.z ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.w ,0);
}


TEST_F(GoalManager, adjustMiningGoalPos)
{
    NavigationGoalManager::GeometryConstraints 
        constraints(
                SAFE_MINING_DISTANCE, 
                MINING_LINE_LENGTH, 
                FINISH_LINE);
    NavigationGoalManager manager(constraints, tfr_msgs::NavigationGoal::TO_MINING);
    geometry_msgs::Pose p;
    p.position.x = 3.4;
    p.position.y = 1;
    p.position.z = 0.4;
    p.orientation.x = .1;
    p.orientation.y = -.3;
    p.orientation.z = -.5;
    p.orientation.w = 1;
    auto nav_goal =  manager.get_updated_mining_goal(p);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.x , 
            SAFE_MINING_DISTANCE);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.y , 1);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.z , 0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.x ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.y ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.z ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.w ,0);
}

TEST_F(GoalManager, adjustMiningGoalBigPos)
{
    NavigationGoalManager::GeometryConstraints 
        constraints(SAFE_MINING_DISTANCE, 
                MINING_LINE_LENGTH, 
                FINISH_LINE);
    NavigationGoalManager manager(constraints, tfr_msgs::NavigationGoal::TO_MINING);
    geometry_msgs::Pose p;
    p.position.x = 3.4;
    p.position.y = 5;
    p.position.z = 0.4;
    p.orientation.x = .1;
    p.orientation.y = -.3;
    p.orientation.z = -.5;
    p.orientation.w = 1;
    auto nav_goal = manager.get_updated_mining_goal(p);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.x , 
            SAFE_MINING_DISTANCE);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.y , 1.015);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.z , 0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.x ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.y ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.z ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.w ,0);
}

TEST_F(GoalManager, adjustMiningGoalBigNeg)
{
    NavigationGoalManager::GeometryConstraints 
        constraints(SAFE_MINING_DISTANCE, 
                MINING_LINE_LENGTH, 
                FINISH_LINE);
    NavigationGoalManager manager(constraints, tfr_msgs::NavigationGoal::TO_MINING);
    geometry_msgs::Pose p;
    p.position.x = 3.4;
    p.position.y = -5;
    p.position.z = 0.4;
    p.orientation.x = .1;
    p.orientation.y = -.3;
    p.orientation.z = -.5;
    p.orientation.w = 1;
    auto nav_goal = manager.get_updated_mining_goal(p);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.x , 
            SAFE_MINING_DISTANCE);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.y , -1.015);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.z , 0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.x ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.y ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.z ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.w ,0);
}

TEST_F(GoalManager, adjustMiningGoalWrongType)
{
    NavigationGoalManager::GeometryConstraints 
        constraints(SAFE_MINING_DISTANCE, 
                MINING_LINE_LENGTH, 
                FINISH_LINE);
    NavigationGoalManager manager(constraints,
            tfr_msgs::NavigationGoal::TO_DUMPING);
    geometry_msgs::Pose p;
    p.position.x = 3.4;
    p.position.y = -5;
    p.position.z = 0.4;
    p.orientation.x = .1;
    p.orientation.y = -.3;
    p.orientation.z = -.5;
    p.orientation.w = 1;
    auto nav_goal = manager.get_updated_mining_goal(p);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.x , 
            FINISH_LINE);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.y , 0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.position.z , 0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.x ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.y ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.z ,0);
    ASSERT_DOUBLE_EQ(nav_goal.target_pose.pose.orientation.w ,0);
    std::cout << "Note: you should have seen a warning when running this" 
        <<" test case, this is defined behavior." << std::endl;
}


int main(int argc, char **argv)
{
    ros::Time::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
