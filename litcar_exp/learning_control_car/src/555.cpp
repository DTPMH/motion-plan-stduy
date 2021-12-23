#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include<geometry_msgs/Twist.h> //运动速度结构体类型  geometry_msgs::Twist的定义文件
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <Eigen/Eigen>
using std::vector;
 nav_msgs::Odometry pot_really;
 geometry_msgs::PoseStamped pot_goal,finally_goal;
 geometry_msgs::Twist vel_cmd; //声明一个geometry_msgs::Twist 类型的对象vel_cmd，并将速度的值赋值到这个对象里面
 ros::Publisher vel_pub,pos_cmd_pub,odom_pub;
 int target=0,arrive=0,my_cout=0;
 float delat_x_path,delat_y_path,d,omega2;
 
 Eigen::Vector3f pt;
 vector<Eigen::Vector3f> pos_pts;

 void pose_cmdcallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
     pt(0)=msg->pose.position.x;
     pt(1)=msg->pose.position.y;
     pt(2)=msg->pose.position.z;
     pos_pts.push_back(pt);
     ROS_INFO("x: %.2f ,y: %.2f ,z: %.2f ",
              pt(0),pt(1),pt(2));
     ROS_INFO("COME IN");
}
void odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  pot_really.pose.pose.position.x=msg->pose.pose.position.x;
  pot_really.pose.pose.position.y=msg->pose.pose.position.y;
  pot_really.pose.pose.position.z=msg->pose.pose.position.z;
  pot_really.pose.pose.orientation.x=msg->pose.pose.orientation.x;
  pot_really.pose.pose.orientation.y=msg->pose.pose.orientation.y;
  pot_really.pose.pose.orientation.z=msg->pose.pose.orientation.z;
  pot_really.pose.pose.orientation.w=msg->pose.pose.orientation.w;  
}
void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if (msg->pose.position.z < -0.1) return;
  finally_goal.pose.position.x=msg->pose.position.x;
  finally_goal.pose.position.y=msg->pose.position.y;
  finally_goal.pose.position.z=msg->pose.position.z;;
  arrive=0;
  target = 1;
  ROS_INFO("receive target ");
}
int main(int argc, char *argv[])
{
    int i=0,t=0;

    ros::init(argc, argv, "vel_ctrl");  //对该节点进行初始化操作
    ros::NodeHandle n;        
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    ros::Subscriber pose_cmd_sub = n.subscribe("/my_position_cmd", 10, pose_cmdcallback);//接收到路径点进行回调函数,赋值引路径点
    ros::Subscriber odom_sub = n.subscribe("/state_ukf/odom", 1, odometryCallback);//接收到小车的定位信息后,调用函数,将小车的姿态传入全局变量中
    ros::Subscriber way_sub = n.subscribe("/my_waypoint", 1, waypointCallback);//接收到目标点的信息后,对目标标志位置1,开始给小车速度指令

    ros::Rate loop_rate(50);//以50hz的速率发送
    while(ros::ok())
    {
        /*omega2=atan2(2*(pot_really.pose.pose.orientation.w*pot_really.pose.pose.orientation.z+pot_really.pose.pose.orientation.x*pot_really.pose.pose.orientation.y),
                       1-2*(pow(pot_really.pose.pose.orientation.y,2)+pow(pot_really.pose.pose.orientation.z,2)));
                       ROS_INFO("OMEGA2 : %.2f",omega2);*/
        ros::spinOnce();
        loop_rate.sleep();
   }
       return 0;
}