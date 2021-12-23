#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include<geometry_msgs/Twist.h> //运动速度结构体类型  geometry_msgs::Twist的定义文件
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Point.h>
#include<nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <vector>
using namespace std;
 Eigen::MatrixXd goal_list;
 nav_msgs::Odometry pot_really;
 geometry_msgs::PoseStamped pot_goal,finally_goal;
 ros::Publisher vel_pub,pos_cmd_pub,odom_pub;
 int target=0,arrive=0,my_cout=0,j=0,i=0;


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vel_ctrl2");  //对该节点进行初始化操作
    ros::NodeHandle n;         //申明一个NodeHandle对象n，并用n生成一个广播对象vel_pub
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    //geometry_msgs::Point pt;
    //Eigen::Vector3d pt;
    Eigen::Vector3f pt=Eigen::Vector3f::Zero();
    vector<Eigen::Vector3f> pos_pts;
    ros::Rate loop_rate(50);//以50hz的速率发送
    while(ros::ok())
    {  
     for (size_t i = 0; i < 10; i++)
     { 
       pt(0)=i;
       pt(1)=i+0.1;
       pt(2)=i+0.2;
       pos_pts.push_back(pt);    
       // ROS_INFO("ok");
      }
      //pos_pts.clear();
     for (size_t t = 0; t < pos_pts.size(); t++)
     {
        ROS_INFO("err: (x %.2f, y %.2f. Z %.2f ) ",
        pos_pts[t](0),
        pos_pts[t](1),
        pos_pts[t](2));
     }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}