#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
 


class Twist_transform
{
public:
  Twist_transform();
  ~Twist_transform();

private:
  void twistCmdCallback(const geometry_msgs::TwistStampedConstPtr& msg);
  ros::Publisher twist_pub_;
  ros::Subscriber  twist_sub_;
    ros::NodeHandle nh;
};

Twist_transform::Twist_transform()
{
    ros::NodeHandle nh;
    twist_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    twist_sub_ = nh.subscribe<geometry_msgs::TwistStamped>("/twist_raw", 10, &Twist_transform::twistCmdCallback,this);
}


Twist_transform::~Twist_transform()
{
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
twist.linear.y = 0;  
twist.linear.z = 0;
        twist.angular.x = 0; 
twist.angular.y = 0; 
twist.angular.z = 0;
    twist_pub_.publish(twist);
}

void Twist_transform::twistCmdCallback(const geometry_msgs::TwistStampedConstPtr& msg)
{
    geometry_msgs::Twist twist = msg->twist;
    twist_pub_.publish(twist);
 
    ROS_INFO("Transform Success !");
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_transform");
    Twist_transform Twist_transform;
    ros::spin();
}
