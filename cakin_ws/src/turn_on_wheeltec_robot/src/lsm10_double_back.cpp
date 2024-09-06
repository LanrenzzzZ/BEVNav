// #include "ros/ros.h"
// #include "sensor_msgs/LaserScan.h"
// #include "std_srvs/Empty.h"
// #include <message_filters/sync_policies/approximate_time.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <ros/ros.h>
// #include <pcl/io/io.h>
// #include <pcl/point_types.h>
// // #include "/home/wheeltec/wheeltec_robot/src/lsm10_v2/lsm10.h"
// #include <turn_on_wheeltec_robot/supersonic.h>
// // #define DEG2RAD(x) ((x)*M_PI/180.) // 角度转弧度
// using namespace std;

// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,
//                                          turn_on_wheeltec_robot::supersonic>
//       syncPolicy;
// std::string frame_id;
// std::string scan_topic_,scan_topic_without_avoid;
// double avoid_A_xoffset,avoid_A_yoffset,avoid_A_rotate,scan_A_angle,
//        avoid_B_xoffset,avoid_B_yoffset,avoid_B_rotate,scan_B_angle,
//        avoid_C_xoffset,avoid_C_yoffset,avoid_C_rotate,scan_C_angle,
//        avoid_D_xoffset,avoid_D_yoffset,avoid_D_rotate,scan_D_angle,
//        avoid_E_xoffset,avoid_E_yoffset,avoid_E_rotate,scan_E_angle,
//        avoid_F_xoffset,avoid_F_yoffset,avoid_F_rotate,scan_F_angle,
//        avoid_distance;
// float angle_max,angle_min,angle_increment,x,y;
// double node_count;
// int count_;
// float distanceA_,distanceB_,distanceC_,distanceD_,distanceE_,distanceF_,distanceG_,distanceH_;
// bool if_avoid_A,if_avoid_B,if_avoid_C,if_avoid_D,if_avoid_E,if_avoid_F,if_avoid_G,if_avoid_H;
// class Lsm10withAvoid{
// public:
//   ros::NodeHandle nh;
//   ros::Publisher Scan_pub;
//   ros::Subscriber Distance_Sub;
//   Lsm10withAvoid();
//   ~Lsm10withAvoid();
//   // void Scan_Callback(const sensor_msgs::LaserScan::ConstPtr &msg,
//   //                  const turn_on_wheeltec_robot::supersonic::ConstPtr &msg_avoid,
//   //                  ros::Publisher *pub);
//   void Distance_Callback(const turn_on_wheeltec_robot::supersonic msg);
// private:
//   // message_filters::Subscriber<turn_on_wheeltec_robot::supersonic> *Distance_Sub;
//   // message_filters::Subscriber<sensor_msgs::LaserScan> *Scan_Sub;
//   // message_filters::Synchronizer<syncPolicy> *sync;
// };

//   Lsm10withAvoid::Lsm10withAvoid(){
//   // ros::Subscriber Scan_Sub;
//   // ros::Subscriber Distance_Sub;
//   // ros::Publisher Scan_pub;
//   ros::NodeHandle nh_private("~");
//     // // 参数获取
//   nh_private.param<std::string>("frame_id",frame_id,"laser");
//   nh_private.param<std::string>("scan_topic_without_avoid",scan_topic_without_avoid,"/scan_without_avoid");
//   nh_private.param<std::string>("scan_topic",scan_topic_,"/scan");

//   nh_private.param<double>("avoid_distance",avoid_distance,0.8);

//   nh_private.param<double>("avoid_A_rotate",avoid_A_rotate,0);
//   nh_private.param<double>("avoid_B_rotate",avoid_B_rotate,0);
//   nh_private.param<double>("avoid_C_rotate",avoid_C_rotate,0);
//   nh_private.param<double>("avoid_D_rotate",avoid_D_rotate,0);
//   nh_private.param<double>("avoid_E_rotate",avoid_E_rotate,0);
//   nh_private.param<double>("avoid_F_rotate",avoid_F_rotate,0);

//   nh_private.param<double>("scan_A_angle",scan_A_angle,45);
//   nh_private.param<double>("scan_B_angle",scan_B_angle,30);
//   nh_private.param<double>("scan_C_angle",scan_C_angle,15);
//   nh_private.param<double>("scan_D_angle",scan_D_angle,345);
//   nh_private.param<double>("scan_E_angle",scan_E_angle,330);
//   nh_private.param<double>("scan_F_angle",scan_F_angle,315);

//   nh_private.param<double>("avoid_A_xoffset",avoid_A_xoffset,0);
//   nh_private.param<double>("avoid_A_yoffset",avoid_A_yoffset,0);

//   nh_private.param<double>("avoid_B_xoffset",avoid_B_xoffset,0);
//   nh_private.param<double>("avoid_B_yoffset",avoid_B_yoffset,0);

//   nh_private.param<double>("avoid_C_xoffset",avoid_C_xoffset,0);
//   nh_private.param<double>("avoid_C_yoffset",avoid_C_yoffset,0);

//   nh_private.param<double>("avoid_D_xoffset",avoid_D_xoffset,0);
//   nh_private.param<double>("avoid_D_yoffset",avoid_D_yoffset,0);

//   nh_private.param<double>("avoid_E_xoffset",avoid_E_xoffset,0);
//   nh_private.param<double>("avoid_E_yoffset",avoid_E_yoffset,0);

//   nh_private.param<double>("avoid_F_xoffset",avoid_F_xoffset,0);
//   nh_private.param<double>("avoid_F_yoffset",avoid_F_yoffset,0);

//   Scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan_topic_, 1000);
//   Distance_Sub= nh.subscribe<turn_on_wheeltec_robot::supersonic>("Distance",100, &Lsm10withAvoid::Distance_Callback,this); 
//   // Scan_Sub= nh.subscribe<sensor_msgs::LaserScan>(scan_topic_without_avoid,100, &Lsm10withAvoid::Scan_Callback,this);

//   // Distance_Sub=new message_filters::Subscriber<turn_on_wheeltec_robot::supersonic> (
//   //     nh, "/Distance", 1, ros::TransportHints().tcpNoDelay());
//   // Scan_Sub=new message_filters::Subscriber<sensor_msgs::LaserScan> (
//   //     nh, scan_topic_without_avoid, 1, ros::TransportHints().tcpNoDelay());

//   // 将两个topic的数据进行同步

//   // sync=new message_filters::Synchronizer<syncPolicy>(syncPolicy(10), *Scan_Sub, *Distance_Sub);
//   // sync->registerCallback(boost::bind(&Lsm10withAvoid::Scan_Callback, this,_1, _2,&Scan_pub));
//   }


//   Lsm10withAvoid::~Lsm10withAvoid(){
//     // delete Distance_Sub;
//     // delete Scan_Sub;
//     // delete sync;
//   }
// void Lsm10withAvoid::Distance_Callback(const turn_on_wheeltec_robot::supersonic msg)
// {
//   if_avoid_A=false;
//   if_avoid_B=false;
//   if_avoid_C=false;
//   if_avoid_D=false;
//   if_avoid_E=false;
//   if_avoid_F=false;
//   if (msg.distanceA<avoid_distance) {distanceA_=msg.distanceA; if_avoid_A=true;}
//   // ROS_INFO("%f\n",distanceA_);
//   if (msg.distanceB<avoid_distance) {distanceB_=msg.distanceB; if_avoid_B=true;}
//   if (msg.distanceB<avoid_distance) {distanceC_=msg.distanceC; if_avoid_C=true;}
//   if (msg.distanceB<avoid_distance) {distanceD_=msg.distanceD; if_avoid_D=true;}
//   if (msg.distanceB<avoid_distance) {distanceE_=msg.distanceE; if_avoid_E=true;}
//   if (msg.distanceB<avoid_distance) {distanceF_=msg.distanceF; if_avoid_F=true;}
//   if (msg.distanceB<avoid_distance) {distanceG_=msg.distanceG; if_avoid_G=true;}
//   if (msg.distanceB<avoid_distance) {distanceH_=msg.distanceH; if_avoid_H=true;}


//   double newPointRangeA;
//   double newPointAx,newPointAy;
//   if(if_avoid_A){
//     newPointAx=avoid_A_xoffset+distanceA_*cos(avoid_A_rotate);
//     newPointAy=avoid_A_yoffset+distanceA_*sin(avoid_A_rotate);
//     newPointRangeA=hypot(newPointAx,newPointAy);
//   }
//   double newPointRangeB;
//   double newPointBx,newPointBy;
//   if(if_avoid_B){
//     newPointBx=avoid_B_xoffset+distanceB_*cos(avoid_B_rotate);
//     newPointBy=avoid_B_yoffset+distanceB_*sin(avoid_B_rotate);
//     newPointRangeB=hypot(newPointBx,newPointBy);
//   }

//   double newPointRangeC;
//   double newPointCx,newPointCy;
//   if(if_avoid_C){
//     newPointCx=avoid_C_xoffset+distanceC_*cos(avoid_C_rotate);
//     newPointCy=avoid_C_yoffset+distanceC_*sin(avoid_C_rotate);
//     newPointRangeC=hypot(newPointCx,newPointCy);
//   }

//   double newPointRangeD;
//   double newPointDx,newPointDy;
//   if(if_avoid_D){
//     newPointDx=avoid_D_xoffset+distanceD_*cos(avoid_D_rotate);
//     newPointDy=avoid_D_yoffset+distanceD_*sin(avoid_D_rotate);
//     newPointRangeD=hypot(newPointDx,newPointDy);
//   }

//   double newPointRangeE;
//   double newPointEx,newPointEy;
//   if(if_avoid_E){
//     newPointEx=avoid_E_xoffset+distanceE_*cos(avoid_E_rotate);
//     newPointEy=avoid_E_yoffset+distanceE_*sin(avoid_E_rotate);
//     newPointRangeE=hypot(newPointEx,newPointEy);
//   }

//   double newPointRangeF;
//   double newPointFx,newPointFy;
//   if(if_avoid_F){
//     newPointFx=avoid_F_xoffset+distanceF_*cos(avoid_F_rotate);
//     newPointFy=avoid_F_yoffset+distanceF_*sin(avoid_F_rotate);
//     newPointRangeF=hypot(newPointFx,newPointFy);
//   }
//   // 创建LaserScan消息
//   angle_min=-3.14;
//   angle_max=3.14;
//   node_count=720;
//   angle_increment=0.008722222;
//   sensor_msgs::LaserScan scan_msg;
//   scan_msg.header.stamp = ros::Time::now();
//   scan_msg.header.frame_id = frame_id;
//   scan_msg.angle_min = angle_min;
//   scan_msg.angle_max = angle_max;
//   scan_msg.angle_increment = angle_increment;
//   scan_msg.scan_time = (ros::Time::now()-scan_msg.header.stamp).toSec();
//   scan_msg.time_increment = scan_msg.scan_time / (double)(node_count);
//   scan_msg.range_min = 0;
//   scan_msg.range_max = 6;
//     //先将数组用inf及0填充
//   scan_msg.ranges.assign(node_count, std::numeric_limits<float>::infinity());
//   scan_msg.intensities.assign(node_count, 0);
//     // int index = round((angle - scan_msg.angle_min) / scan_msg.angle_increment); // 当前扫描点的索引号
//     int indexD = round(DEG2RAD(scan_D_angle)/ scan_msg.angle_increment);
//     // ROS_INFO("indexD:%d\n",indexD);
//     int indexE = round(DEG2RAD(scan_E_angle)/ scan_msg.angle_increment);
//     // ROS_INFO("indexE:%d\n",indexE);

//     int indexF = round(DEG2RAD(scan_F_angle)/ scan_msg.angle_increment);
//     // ROS_INFO("indexF:%d\n",indexF);

//     int indexA = round(DEG2RAD(scan_A_angle)/ scan_msg.angle_increment);
//     // ROS_INFO("indexA:%d\n",indexA);

//     int indexB = round(DEG2RAD(scan_B_angle)/ scan_msg.angle_increment);
//     // ROS_INFO("indexB:%d\n",indexB);

//     int indexC = round(DEG2RAD(scan_C_angle)/ scan_msg.angle_increment);
//     // ROS_INFO("111111111111");
//   for (int i=0;i<node_count;i++)
//   {   
//     // ROS_INFO("1111");
//       if(if_avoid_C)
//       {
//         if(i>=0 && i<indexC)
//         {
//           scan_msg.ranges[i] = newPointRangeC;
//         }
//       }

//       if(if_avoid_B)
//       {
//         if(i>=indexC && i<indexB)
//         {
//           scan_msg.ranges[i] = newPointRangeB;
//         }
//       }

//       if(if_avoid_A)
//       {
//         // ROS_INFO("AAAAAAAAAAA");
//         if(i>=indexB && i<indexA)
//         {
//           scan_msg.ranges[i] = newPointRangeA;
//         }
//       }

//       if(if_avoid_F)
//       {
//         if(i>=indexF && i<indexE)
//         {
//           scan_msg.ranges[i] = newPointRangeF;
//         }
//       }

//       if(if_avoid_E)
//       {
//         if(i>=indexE && i<indexD)
//         {
//           scan_msg.ranges[i] = newPointRangeE;
//         }
//       }
      
//       if(if_avoid_D)
//       {
//         if(i>=indexD && i<node_count)
//         {
//           scan_msg.ranges[i] = newPointRangeD;
//         }
//       }

//   }
// Scan_pub.publish(scan_msg);



// }

// // void Lsm10withAvoid::Scan_Callback(const sensor_msgs::LaserScan::ConstPtr &msg,
// //                    const turn_on_wheeltec_robot::supersonic::ConstPtr &msg_avoid,
// //                    ros::Publisher *pub)
// // {
// //   if_avoid_A=false;
// //   if_avoid_B=false;
// //   if_avoid_C=false;
// //   if_avoid_D=false;
// //   if_avoid_E=false;
// //   if_avoid_F=false;
// //   if (msg_avoid->distanceA<avoid_distance) {distanceA_=msg_avoid->distanceA; if_avoid_A=true;}
// //   // ROS_INFO("%f\n",avoid_distance);
// //   if (msg_avoid->distanceB<avoid_distance) {distanceB_=msg_avoid->distanceB; if_avoid_B=true;}
// //   if (msg_avoid->distanceC<avoid_distance) {distanceC_=msg_avoid->distanceC; if_avoid_C=true;}
// //   if (msg_avoid->distanceD<avoid_distance) {distanceD_=msg_avoid->distanceD; if_avoid_D=true;}
// //   if (msg_avoid->distanceE<avoid_distance) {distanceE_=msg_avoid->distanceE; if_avoid_E=true;}
// //   if (msg_avoid->distanceF<avoid_distance) {distanceF_=msg_avoid->distanceF; if_avoid_F=true;}
// //   if (msg_avoid->distanceG<avoid_distance) {distanceG_=msg_avoid->distanceG; if_avoid_G=true;}
// //   if (msg_avoid->distanceH<avoid_distance) {distanceH_=msg_avoid->distanceH; if_avoid_H=true;}

// //   ros::Subscriber Scan_Sub;
// //   angle_max = msg->angle_max;
// //   angle_min = msg->angle_min;
// //   angle_increment = msg->angle_increment;
// //   node_count = round((angle_max-angle_min)/angle_increment);
// //   // std::cout<<node_count<<std::endl;
// //   // 创建点云
// //   pcl::PointCloud<pcl::PointXYZI> pointcloud;
// //   pcl::PointXYZI newPoint; // 创建一个点
// //   newPoint.z = 0.0;
// //   double newPointAngle;

// //   double newPointRangeA;
// //   double newPointAx,newPointAy;
// //   if(if_avoid_A){
// //     newPointAx=avoid_A_xoffset+distanceA_*cos(avoid_A_rotate);
// //     newPointAy=avoid_A_yoffset+distanceA_*sin(avoid_A_rotate);
// //     newPointRangeA=hypot(newPointAx,newPointAy);
// //   }
// //   double newPointRangeB;
// //   double newPointBx,newPointBy;
// //   if(if_avoid_B){
// //     newPointBx=avoid_B_xoffset+distanceB_*cos(avoid_B_rotate);
// //     newPointBy=avoid_B_yoffset+distanceB_*sin(avoid_B_rotate);
// //     newPointRangeB=hypot(newPointBx,newPointBy);
// //   }

// //   double newPointRangeC;
// //   double newPointCx,newPointCy;
// //   if(if_avoid_C){
// //     newPointCx=avoid_C_xoffset+distanceC_*cos(avoid_C_rotate);
// //     newPointCy=avoid_C_yoffset+distanceC_*sin(avoid_C_rotate);
// //     newPointRangeC=hypot(newPointCx,newPointCy);
// //   }

// //   double newPointRangeD;
// //   double newPointDx,newPointDy;
// //   if(if_avoid_D){
// //     newPointDx=avoid_D_xoffset+distanceD_*cos(avoid_D_rotate);
// //     newPointDy=avoid_D_yoffset+distanceD_*sin(avoid_D_rotate);
// //     newPointRangeD=hypot(newPointDx,newPointDy);
// //   }

// //   double newPointRangeE;
// //   double newPointEx,newPointEy;
// //   if(if_avoid_E){
// //     newPointEx=avoid_E_xoffset+distanceE_*cos(avoid_E_rotate);
// //     newPointEy=avoid_E_yoffset+distanceE_*sin(avoid_E_rotate);
// //     newPointRangeE=hypot(newPointEx,newPointEy);
// //   }

// //   double newPointRangeF;
// //   double newPointFx,newPointFy;
// //   if(if_avoid_F){
// //     newPointFx=avoid_F_xoffset+distanceF_*cos(avoid_F_rotate);
// //     newPointFy=avoid_F_yoffset+distanceF_*sin(avoid_F_rotate);
// //     newPointRangeF=hypot(newPointFx,newPointFy);
// //   }
// //     //将scan1的消息转换为点云
// //   for (int i=0;i<node_count;i++)
// //   {   
// //       //如果没有障碍物则跳过
// //       if(msg->ranges[i]==std::numeric_limits<float>::infinity())
// //       {
// //         continue;
// //       }
// //       newPointAngle = msg->angle_min + msg->angle_increment * i;
// //       newPoint.x = msg->ranges[i] * cos(newPointAngle);
// //       newPoint.y = msg->ranges[i] * sin(newPointAngle);
// //       newPoint.intensity = msg->intensities[i];
// //       pointcloud.push_back(newPoint);
// //   }
// //   // 创建LaserScan消息
// //   sensor_msgs::LaserScan scan_msg;
// //   scan_msg.header.stamp = ros::Time::now();
// //   scan_msg.header.frame_id = frame_id;
// //   scan_msg.angle_min = angle_min;
// //   scan_msg.angle_max = angle_max;
// //   scan_msg.angle_increment = angle_increment;
// //   scan_msg.scan_time = (ros::Time::now()-scan_msg.header.stamp).toSec();
// //   scan_msg.time_increment = scan_msg.scan_time / (double)(node_count);
// //   scan_msg.range_min = msg->range_min;
// //   scan_msg.range_max = msg->range_max;
// //     //先将数组用inf及0填充
// //   scan_msg.ranges.assign(node_count, std::numeric_limits<float>::infinity());
// //   scan_msg.intensities.assign(node_count, 0);
// //   for (auto point : pointcloud.points)
// //   {
// //     float range = hypot(point.x, point.y);
// //     float angle = atan2(point.y, point.x);
// //     int index = round((angle - scan_msg.angle_min) / scan_msg.angle_increment); // 当前扫描点的索引号
// //     int indexD = round(DEG2RAD(scan_D_angle)/ scan_msg.angle_increment);
// //     // ROS_INFO("indexD:%d\n",indexD);
// //     int indexE = round(DEG2RAD(scan_E_angle)/ scan_msg.angle_increment);
// //     // ROS_INFO("indexE:%d\n",indexE);

// //     int indexF = round(DEG2RAD(scan_F_angle)/ scan_msg.angle_increment);
// //     // ROS_INFO("indexF:%d\n",indexF);

// //     int indexA = round(DEG2RAD(scan_A_angle)/ scan_msg.angle_increment);
// //     // ROS_INFO("indexA:%d\n",indexA);

// //     int indexB = round(DEG2RAD(scan_B_angle)/ scan_msg.angle_increment);
// //     // ROS_INFO("indexB:%d\n",indexB);

// //     int indexC = round(DEG2RAD(scan_C_angle)/ scan_msg.angle_increment);
// //     // ROS_INFO("indexC:%d\n",indexC);

// //     if (index >= 0 && index < node_count)
// //     {
// //       //如果range小于range[index]则赋值
// //       // ROS_INFO("index:%d\n",index);
// //       if(range<scan_msg.ranges[index])
// //       {
// //         scan_msg.ranges[index] = range;
// //         point.intensity = 0 ; //M10 have no intensity
// //         scan_msg.intensities[index] = point.intensity;
// //       }

// //       if(if_avoid_C)
// //       {
// //         if(index>=0 && index<indexC)
// //         {
// //           scan_msg.ranges[index] = (range<=newPointRangeC)?range:newPointRangeC;
// //         }
// //       }

// //       if(if_avoid_B)
// //       {
// //         if(index>=indexC && index<indexB)
// //         {
// //           scan_msg.ranges[index] = (range<=newPointRangeB)?range:newPointRangeB;
// //         }
// //       }

// //       if(if_avoid_A)
// //       {
// //         if(index>=indexB && index<indexA)
// //         {
// //           scan_msg.ranges[index] = (range<=newPointRangeA)?range:newPointRangeA;
// //         }
// //       }

// //       if(if_avoid_F)
// //       {
// //         if(index>=indexF && index<indexE)
// //         {
// //           scan_msg.ranges[index] = (range<=newPointRangeF)?range:newPointRangeF;
// //         }
// //       }

// //       if(if_avoid_E)
// //       {
// //         if(index>=indexE && index<indexD)
// //         {
// //           scan_msg.ranges[index] = (range<=newPointRangeE)?range:newPointRangeE;
// //         }
// //       }
      
// //       if(if_avoid_D)
// //       {
// //         if(index>=indexD && index<node_count)
// //         {
// //           scan_msg.ranges[index] = (range<=newPointRangeD)?range:newPointRangeD;
// //         }
// //       }






// //     }
// //   }
// //   // Scan_pub.publish(scan_msg);
// //   pub->publish(scan_msg);
// // }


// int main(int argc, char **argv) {
//   // 节点初始化

//   ros::init(argc, argv, "lsm10_v2_double_with_avoid");
//   ROS_INFO("lsm10_v2_double_with_avoid init");


//   Lsm10withAvoid Lsm10withAvoid;
//   ros::spin();

//   return 0;
// }


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
// #include "/home/wheeltec/wheeltec_robot/src/lsm10_v2/lsm10.h"
#include <turn_on_wheeltec_robot/supersonic.h>
// #define DEG2RAD(x) ((x)*M_PI/180.) // 角度转弧度
using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,
                                         turn_on_wheeltec_robot::supersonic>
      syncPolicy;
std::string frame_id;
std::string scan_topic_,scan_topic_without_avoid;
double avoid_A_xoffset,avoid_A_yoffset,avoid_A_rotate,scan_A_angle,
       avoid_B_xoffset,avoid_B_yoffset,avoid_B_rotate,scan_B_angle,
       avoid_C_xoffset,avoid_C_yoffset,avoid_C_rotate,scan_C_angle,
       avoid_D_xoffset,avoid_D_yoffset,avoid_D_rotate,scan_D_angle,
       avoid_E_xoffset,avoid_E_yoffset,avoid_E_rotate,scan_E_angle,
       avoid_F_xoffset,avoid_F_yoffset,avoid_F_rotate,scan_F_angle,
       avoid_distance;
float angle_max,angle_min,angle_increment,x,y;
size_t node_count;
int count_;
float distanceA_,distanceB_,distanceC_,distanceD_,distanceE_,distanceF_,distanceG_,distanceH_;
bool if_avoid_A,if_avoid_B,if_avoid_C,if_avoid_D,if_avoid_E,if_avoid_F,if_avoid_G,if_avoid_H;
class Lsm10withAvoid{
public:
  ros::NodeHandle nh;
  ros::Publisher Scan_pub;
  
  Lsm10withAvoid();
  ~Lsm10withAvoid();
  void Scan_Callback(const sensor_msgs::LaserScan::ConstPtr &msg,
                   const turn_on_wheeltec_robot::supersonic::ConstPtr &msg_avoid,
                   ros::Publisher *pub);
private:
  message_filters::Subscriber<turn_on_wheeltec_robot::supersonic> *Distance_Sub;
  message_filters::Subscriber<sensor_msgs::LaserScan> *Scan_Sub;
  message_filters::Synchronizer<syncPolicy> *sync;
};

  Lsm10withAvoid::Lsm10withAvoid(){
  // ros::Subscriber Scan_Sub;
  // ros::Subscriber Distance_Sub;
  ros::NodeHandle nh_private("~");
    // // 参数获取
  nh_private.param<std::string>("frame_id",frame_id,"laser");
  nh_private.param<std::string>("scan_topic_without_avoid",scan_topic_without_avoid,"/scan_without_avoid");
  nh_private.param<std::string>("scan_topic",scan_topic_,"/scan");

  nh_private.param<double>("avoid_distance",avoid_distance,0.8);

  nh_private.param<double>("avoid_A_rotate",avoid_A_rotate,0);
  nh_private.param<double>("avoid_B_rotate",avoid_B_rotate,0);
  nh_private.param<double>("avoid_C_rotate",avoid_C_rotate,0);
  nh_private.param<double>("avoid_D_rotate",avoid_D_rotate,0);
  nh_private.param<double>("avoid_E_rotate",avoid_E_rotate,0);
  nh_private.param<double>("avoid_F_rotate",avoid_F_rotate,0);

  nh_private.param<double>("scan_A_angle",scan_A_angle,45);
  nh_private.param<double>("scan_B_angle",scan_B_angle,30);
  nh_private.param<double>("scan_C_angle",scan_C_angle,15);
  nh_private.param<double>("scan_D_angle",scan_D_angle,345);
  nh_private.param<double>("scan_E_angle",scan_E_angle,330);
  nh_private.param<double>("scan_F_angle",scan_F_angle,315);

  nh_private.param<double>("avoid_A_xoffset",avoid_A_xoffset,0);
  nh_private.param<double>("avoid_A_yoffset",avoid_A_yoffset,0);

  nh_private.param<double>("avoid_B_xoffset",avoid_B_xoffset,0);
  nh_private.param<double>("avoid_B_yoffset",avoid_B_yoffset,0);

  nh_private.param<double>("avoid_C_xoffset",avoid_C_xoffset,0);
  nh_private.param<double>("avoid_C_yoffset",avoid_C_yoffset,0);

  nh_private.param<double>("avoid_D_xoffset",avoid_D_xoffset,0);
  nh_private.param<double>("avoid_D_yoffset",avoid_D_yoffset,0);

  nh_private.param<double>("avoid_E_xoffset",avoid_E_xoffset,0);
  nh_private.param<double>("avoid_E_yoffset",avoid_E_yoffset,0);

  nh_private.param<double>("avoid_F_xoffset",avoid_F_xoffset,0);
  nh_private.param<double>("avoid_F_yoffset",avoid_F_yoffset,0);

  Scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan_topic_, 1000);
  // Distance_Sub= nh.subscribe<turn_on_wheeltec_robot::supersonic>("Distance",100, &Lsm10withAvoid::Distance_Callback,this); 
  // Scan_Sub= nh.subscribe<sensor_msgs::LaserScan>(scan_topic_without_avoid,100, &Lsm10withAvoid::Scan_Callback,this);

  Distance_Sub=new message_filters::Subscriber<turn_on_wheeltec_robot::supersonic> (
      nh, "/Distance", 1, ros::TransportHints().tcpNoDelay());
  Scan_Sub=new message_filters::Subscriber<sensor_msgs::LaserScan> (
      nh, scan_topic_without_avoid, 1, ros::TransportHints().tcpNoDelay());

  // 将两个topic的数据进行同步

  sync=new message_filters::Synchronizer<syncPolicy>(syncPolicy(10), *Scan_Sub, *Distance_Sub);
  sync->registerCallback(boost::bind(&Lsm10withAvoid::Scan_Callback, this,_1, _2,&Scan_pub));
  }


  Lsm10withAvoid::~Lsm10withAvoid(){
    delete Distance_Sub;
    delete Scan_Sub;
    delete sync;
  }
// void Distance_Callback(const turn_on_wheeltec_robot::supersonic msg)
// {
//   if_avoid_A=false;
//   if_avoid_B=false;
//   if_avoid_C=false;
//   if_avoid_D=false;
//   if_avoid_E=false;
//   if_avoid_F=false;
//   if (msg.distanceA<avoid_distance) {distanceA_=msg.distanceA; if_avoid_A=true;}
//   ROS_INFO("%f\n",avoid_distance);
//   if (msg.distanceB<avoid_distance) {distanceB_=msg.distanceB; if_avoid_B=true;}
//   if (msg.distanceB<avoid_distance) {distanceC_=msg.distanceC; if_avoid_C=true;}
//   if (msg.distanceB<avoid_distance) {distanceD_=msg.distanceD; if_avoid_D=true;}
//   if (msg.distanceB<avoid_distance) {distanceE_=msg.distanceE; if_avoid_E=true;}
//   if (msg.distanceB<avoid_distance) {distanceF_=msg.distanceF; if_avoid_F=true;}
//   if (msg.distanceB<avoid_distance) {distanceG_=msg.distanceG; if_avoid_G=true;}
//   if (msg.distanceB<avoid_distance) {distanceH_=msg.distanceH; if_avoid_H=true;}
// }

void Lsm10withAvoid::Scan_Callback(const sensor_msgs::LaserScan::ConstPtr &msg,
                   const turn_on_wheeltec_robot::supersonic::ConstPtr &msg_avoid,
                   ros::Publisher *pub)
{
  // if_avoid_A=false;
  // if_avoid_B=false;
  // if_avoid_C=false;
  // if_avoid_D=false;
  // if_avoid_E=false;
  // if_avoid_F=false;
  // if (msg_avoid->distanceA<avoid_distance) {distanceA_=msg_avoid->distanceA; if_avoid_A=true;}
  // // ROS_INFO("%f\n",avoid_distance);
  // if (msg_avoid->distanceB<avoid_distance) {distanceB_=msg_avoid->distanceB; if_avoid_B=true;}
  // if (msg_avoid->distanceC<avoid_distance) {distanceC_=msg_avoid->distanceC; if_avoid_C=true;}
  // if (msg_avoid->distanceD<avoid_distance) {distanceD_=msg_avoid->distanceD; if_avoid_D=true;}
  // if (msg_avoid->distanceE<avoid_distance) {distanceE_=msg_avoid->distanceE; if_avoid_E=true;}
  // if (msg_avoid->distanceF<avoid_distance) {distanceF_=msg_avoid->distanceF; if_avoid_F=true;}
  // if (msg_avoid->distanceG<avoid_distance) {distanceG_=msg_avoid->distanceG; if_avoid_G=true;}
  // if (msg_avoid->distanceH<avoid_distance) {distanceH_=msg_avoid->distanceH; if_avoid_H=true;}

  distanceA_=msg_avoid->distanceA;
  distanceB_=msg_avoid->distanceB;
  distanceC_=msg_avoid->distanceC;
  distanceD_=msg_avoid->distanceD;
  distanceE_=msg_avoid->distanceE;
  distanceF_=msg_avoid->distanceF;
  distanceG_=msg_avoid->distanceG;
  distanceH_=msg_avoid->distanceH;

  ros::Subscriber Scan_Sub;
  angle_max = msg->angle_max;
  angle_min = msg->angle_min;
  angle_increment = msg->angle_increment;
  node_count = round((angle_max-angle_min)/angle_increment);
  // std::cout<<node_count<<std::endl;
  // 创建点云
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  pcl::PointXYZI newPoint; // 创建一个点
  newPoint.z = 0.0;
  double newPointAngle;

  double newPointRangeA;
  double newPointAx,newPointAy;
  newPointAx=avoid_A_xoffset+distanceA_*cos(avoid_A_rotate);
  newPointAy=avoid_A_yoffset+distanceA_*sin(avoid_A_rotate);
  newPointRangeA=hypot(newPointAx,newPointAy);
  double newPointRangeB;
  double newPointBx,newPointBy;
  newPointBx=avoid_B_xoffset+distanceB_*cos(avoid_B_rotate);
  newPointBy=avoid_B_yoffset+distanceB_*sin(avoid_B_rotate);
  newPointRangeB=hypot(newPointBx,newPointBy);
  double newPointRangeC;
  double newPointCx,newPointCy;
  newPointCx=avoid_C_xoffset+distanceC_*cos(avoid_C_rotate);
  newPointCy=avoid_C_yoffset+distanceC_*sin(avoid_C_rotate);
  newPointRangeC=hypot(newPointCx,newPointCy);
  double newPointRangeD;
  double newPointDx,newPointDy;
  newPointDx=avoid_D_xoffset+distanceD_*cos(avoid_D_rotate);
  newPointDy=avoid_D_yoffset+distanceD_*sin(avoid_D_rotate);
  newPointRangeD=hypot(newPointDx,newPointDy);
  double newPointRangeE;
  double newPointEx,newPointEy;
  newPointEx=avoid_E_xoffset+distanceE_*cos(avoid_E_rotate);
  newPointEy=avoid_E_yoffset+distanceE_*sin(avoid_E_rotate);
  newPointRangeE=hypot(newPointEx,newPointEy);
  double newPointRangeF;
  double newPointFx,newPointFy;
  newPointFx=avoid_F_xoffset+distanceF_*cos(avoid_F_rotate);
  newPointFy=avoid_F_yoffset+distanceF_*sin(avoid_F_rotate);
  newPointRangeF=hypot(newPointFx,newPointFy);
    //将scan1的消息转换为点云
  for (int i=0;i<node_count;i++)
  {   
      //如果没有障碍物则跳过
      if(msg->ranges[i]==std::numeric_limits<float>::infinity())
      {
        continue;
      }
      newPointAngle = msg->angle_min + msg->angle_increment * i;
      newPoint.x = msg->ranges[i] * cos(newPointAngle);
      newPoint.y = msg->ranges[i] * sin(newPointAngle);
      newPoint.intensity = msg->intensities[i];
      pointcloud.push_back(newPoint);
  }
  // 创建LaserScan消息
  sensor_msgs::LaserScan scan_msg;
  scan_msg.header.stamp = ros::Time::now();
  scan_msg.header.frame_id = frame_id;
  scan_msg.angle_min = angle_min;
  scan_msg.angle_max = angle_max;
  scan_msg.angle_increment = angle_increment;
  scan_msg.scan_time = (ros::Time::now()-scan_msg.header.stamp).toSec();
  scan_msg.time_increment = scan_msg.scan_time / (double)(node_count);
  scan_msg.range_min = msg->range_min;
  scan_msg.range_max = msg->range_max;
    //先将数组用inf及0填充
  scan_msg.ranges.assign(node_count, std::numeric_limits<float>::infinity());
  scan_msg.intensities.assign(node_count, 0);
  int indexD = round(DEG2RAD(scan_D_angle)/ scan_msg.angle_increment);
  int indexE = round(DEG2RAD(scan_E_angle)/ scan_msg.angle_increment);
  int indexF = round(DEG2RAD(scan_F_angle)/ scan_msg.angle_increment);
  int indexA = round(DEG2RAD(scan_A_angle)/ scan_msg.angle_increment);
  int indexB = round(DEG2RAD(scan_B_angle)/ scan_msg.angle_increment);
  int indexC = round(DEG2RAD(scan_C_angle)/ scan_msg.angle_increment);
  for (auto point : pointcloud.points)
  {
    float range = hypot(point.x, point.y);
    float angle = atan2(point.y, point.x);
    int index = round((angle - scan_msg.angle_min) / scan_msg.angle_increment); // 当前扫描点的索引号
    // ROS_INFO("indexC:%d\n",indexC);

    if (index >= 0 && index < node_count)
    {
      //如果range小于range[index]则赋值
      // ROS_INFO("index:%d\n",index);
      if(range<scan_msg.ranges[index])
      {
        scan_msg.ranges[index] = range;
        point.intensity = 0 ; //M10 have no intensity
        scan_msg.intensities[index] = point.intensity;
      }

      if(distanceC_<avoid_distance)
      {
        if(index>=0 && index<indexC)
        {
          scan_msg.ranges[index] = (range<=newPointRangeC)?range:newPointRangeC;
        }
      }

      if(distanceB_<avoid_distance)
      {
        if(index>=indexC && index<indexB)
        {
          scan_msg.ranges[index] = (range<=newPointRangeB)?range:newPointRangeB;
        }
      }

      if(distanceA_<avoid_distance)
      {
        if(index>=indexB && index<indexA)
        {
          scan_msg.ranges[index] = (range<=newPointRangeA)?range:newPointRangeA;
        }
      }

      if(distanceF_<avoid_distance)
      {
        if(index>=indexF && index<indexE)
        {
          scan_msg.ranges[index] = (range<=newPointRangeF)?range:newPointRangeF;
        }
      }

      if(distanceE_<avoid_distance)
      {
        if(index>=indexE && index<indexD)
        {
          scan_msg.ranges[index] = (range<=newPointRangeE)?range:newPointRangeE;
        }
      }
      
      if(distanceD_<avoid_distance)
      {
        if(index>=indexD && index<node_count)
        {
          scan_msg.ranges[index] = (range<=newPointRangeD)?range:newPointRangeD;
        }
      }
    }
  }
  // Scan_pub.publish(scan_msg);
  pub->publish(scan_msg);
}


int main(int argc, char **argv) {
  // 节点初始化

  ros::init(argc, argv, "lsm10_v2_double_with_avoid");
  ROS_INFO("lsm10_v2_double_with_avoid init");


  Lsm10withAvoid Lsm10withAvoid;
  ros::spin();

  return 0;
}
