#include "ros/ros.h"
#include <iostream>

#include<geometry_msgs/Twist.h> //运动速度结构体类型  geometry_msgs::Twist的定义文件


 geometry_msgs::Twist vel_cmd; //声明一个geometry_msgs::Twist 类型的对象vel_cmd，并将速度的值赋值到这个对象里面
 ros::Publisher vel_pub;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vel_ctrl");  //对该节点进行初始化操作
    ros::NodeHandle n;        
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Rate loop_rate(50);//以50hz的速率发送
    while(ros::ok())
    {
     vel_cmd.linear.x=0.3;
     vel_cmd.linear.y=-0.3;
     vel_cmd.linear.z=0;

     vel_cmd.angular.x=0;
     vel_cmd.angular.y=0;
     vel_cmd.angular.z=1.5;

     vel_pub.publish(vel_cmd);
     ros::spinOnce();
     loop_rate.sleep();
   }
       return 0;
}