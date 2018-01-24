#include <gtest/gtest.h>
#include <navigation_goal_manager.h>
#include <tfr_msgs/NavigationAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <iostream>

TEST(GoalManager, miningDistance)
{
    ASSERT_TRUE(NavigationGoalManager::SAFE_MINING_DISTANCE > 0.66);
    ASSERT_TRUE(NavigationGoalManager::SAFE_MINING_DISTANCE < 5.75);
}

TEST(GoalManager, miningLineLength)
{
    ASSERT_TRUE(NavigationGoalManager::MINING_LINE_LENGTH > 0);
    ASSERT_TRUE(NavigationGoalManager::MINING_LINE_LENGTH <= 2.03);
}

TEST(GoalManager, finishLineLength)
{
    ASSERT_TRUE(NavigationGoalManager::FINISH_LINE > 0.66);
    ASSERT_TRUE(NavigationGoalManager::FINISH_LINE <= 6.84);
}

TEST(GoalManager, defaultConstructor)
{
    NavigationGoalManager manager{};
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.x, 0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.y, 0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.z , 0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.x ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.y ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.z ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.w ,0);
}

TEST(GoalManager, constructorAndInitializer)
{
    NavigationGoalManager manager(tfr_msgs::NavigationGoal::TO_MINING);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.x ,
            NavigationGoalManager::SAFE_MINING_DISTANCE);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.y , 0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.z , 0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.x ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.y ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.z ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.w ,0);
    manager.location_code = tfr_msgs::NavigationGoal::TO_DUMPING;
    manager.initialize_goal();
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.x ,
            NavigationGoalManager::FINISH_LINE);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.y , 0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.z , 0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.x ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.y ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.z ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.w ,0);
}

TEST(GoalManager, adjustMiningGoalNeg)
{
    NavigationGoalManager manager(tfr_msgs::NavigationGoal::TO_MINING);
    geometry_msgs::Pose p;
    p.position.x = 3.4;
    p.position.y = -.05;
    p.position.z = 0.4;
    p.orientation.x = .1;
    p.orientation.y = -.3;
    p.orientation.z = -.5;
    p.orientation.w = 1;
    manager.update_mining_goal(p);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.x ,
            NavigationGoalManager::SAFE_MINING_DISTANCE);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.y , -.05);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.z , 0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.x ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.y ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.z ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.w ,0);
}


TEST(GoalManager, adjustMiningGoalPos)
{
    NavigationGoalManager manager(tfr_msgs::NavigationGoal::TO_MINING);
    geometry_msgs::Pose p;
    p.position.x = 3.4;
    p.position.y = 1;
    p.position.z = 0.4;
    p.orientation.x = .1;
    p.orientation.y = -.3;
    p.orientation.z = -.5;
    p.orientation.w = 1;
    manager.update_mining_goal(p);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.x ,
            NavigationGoalManager::SAFE_MINING_DISTANCE);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.y , 1);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.z , 0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.x ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.y ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.z ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.w ,0);
}

TEST(GoalManager, adjustMiningGoalBigPos)
{
    NavigationGoalManager manager(tfr_msgs::NavigationGoal::TO_MINING);
    geometry_msgs::Pose p;
    p.position.x = 3.4;
    p.position.y = 5;
    p.position.z = 0.4;
    p.orientation.x = .1;
    p.orientation.y = -.3;
    p.orientation.z = -.5;
    p.orientation.w = 1;
    manager.update_mining_goal(p);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.x ,
            NavigationGoalManager::SAFE_MINING_DISTANCE);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.y , 1.015);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.z , 0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.x ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.y ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.z ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.w ,0);
}

TEST(GoalManager, adjustMiningGoalBigNeg)
{
    NavigationGoalManager manager(tfr_msgs::NavigationGoal::TO_MINING);
    geometry_msgs::Pose p;
    p.position.x = 3.4;
    p.position.y = -5;
    p.position.z = 0.4;
    p.orientation.x = .1;
    p.orientation.y = -.3;
    p.orientation.z = -.5;
    p.orientation.w = 1;
    manager.update_mining_goal(p);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.x ,
            NavigationGoalManager::SAFE_MINING_DISTANCE);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.y , -1.015);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.z , 0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.x ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.y ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.z ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.w ,0);
}

TEST(GoalManager, adjustMiningGoalWrongType)
{
    NavigationGoalManager manager(tfr_msgs::NavigationGoal::TO_DUMPING);
    geometry_msgs::Pose p;
    p.position.x = 3.4;
    p.position.y = -5;
    p.position.z = 0.4;
    p.orientation.x = .1;
    p.orientation.y = -.3;
    p.orientation.z = -.5;
    p.orientation.w = 1;
    manager.update_mining_goal(p);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.x ,
            NavigationGoalManager::FINISH_LINE);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.y , 0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.position.z , 0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.x ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.y ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.z ,0);
    ASSERT_DOUBLE_EQ(manager.nav_goal.target_pose.pose.orientation.w ,0);
    std::cout << "Note: you should have seen a warning when running this" 
        <<" test case, this is defined behavior." << std::endl;
}


int main(int argc, char **argv)
{
    ros::Time::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
