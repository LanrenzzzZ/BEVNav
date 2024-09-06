#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
using namespace std;
 
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    MoveBaseClient ac("move_base", true);
 
    ac.waitForServer();
    move_base_msgs::MoveBaseGoal goal;
 
    //待发送的 目标点 在 map 坐标系下的坐标位置
    goal.target_pose.pose.position.x = 1;
    goal.target_pose.pose.position.y = 1;
    goal.target_pose.pose.orientation.z = 0.374;
    goal.target_pose.pose.orientation.w = 0.92;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
     
    ROS_INFO(" Init success!!! ");
    ac.sendGoal(goal);
    // ROS_INFO("Send Goal !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
    // else
    // {
    //     ROS_WARN("The Goal Planning Failed for some reason");
    // }
    // return 0;
}