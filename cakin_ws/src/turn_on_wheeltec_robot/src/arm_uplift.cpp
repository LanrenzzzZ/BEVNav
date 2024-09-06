#include <ros/ros.h>
#include <ros/console.h>
#include <industrial_msgs/RobotStatus.h>
#include <industrial_msgs/RobotMode.h>
#include <moveit/move_group_interface/move_group_interface.h>

int arm_state=0;//arm_state为1机械臂已正常开启
void Robot_Status_Callback(industrial_msgs::RobotStatus robotstatus)
{
    industrial_msgs::RobotMode mode_=robotstatus.mode;
    industrial_msgs::TriState motion_possible_=robotstatus.motion_possible;
    industrial_msgs::TriState drives_powered_=robotstatus.drives_powered;
    industrial_msgs::TriState e_stopped_=robotstatus.e_stopped;
    if(mode_.val== -1 )
    {
        //启动机械臂
        ROS_INFO_STREAM_ONCE("Moveit is disable,waiting...");
        system("rosservice call /system_service/enable '{}' ");
    }
    if(mode_.val==0 && e_stopped_.val==1)
    {
        ROS_WARN_STREAM_ONCE("Moveit e_stopped,Please turn on manually");
    }
    if(mode_.val== 0 )
    {
        if(motion_possible_.val==1 && drives_powered_.val==1)
        {
            sleep(4);//确保机械臂已启动完毕
            ROS_INFO_STREAM_ONCE("Moveit is enable");
            arm_state =1;
        }
        return;
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_uplift");
    ros::NodeHandle n; //Create a node handle //创建节点句柄
    ros::Subscriber Robot_Status_Sub=n.subscribe("/robot_status",10, Robot_Status_Callback); 
    while(ros::ok())
    {
        if(arm_state==1)
        {
            ros::AsyncSpinner spinner(2);
            spinner.start();
            moveit::planning_interface::MoveGroupInterface manipulator("manipulator");
            manipulator.allowReplanning(true);//当运动规划失败后，允许重新规划
            // 控制机械臂回到uplift
            manipulator.setNamedTarget("uplift");
            manipulator.move();
            sleep(4); 
            ROS_INFO_STREAM_ONCE("Arm uplift success");
            spinner.stop();
            ros::shutdown(); 
            return 0; 
        }
        ros::spinOnce();
    }
}
