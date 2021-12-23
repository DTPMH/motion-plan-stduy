#include "ros/ros.h"
#include <iostream>
#include <math.h>

#include<geometry_msgs/Twist.h> //运动速度结构体类型  geometry_msgs::Twist的定义文件

#include "control_car/PositionCommand.h"

 geometry_msgs::Twist vel_cmd; //声明一个geometry_msgs::Twist 类型的对象vel_cmd，并将速度的值赋值到这个对象里面
 ros::Publisher vel_pub;
 float Vx,Vy,V;

void cmdCallback(const control_car::PositionCommand::ConstPtr& msg) {
     Vx=msg->velocity.x;
     Vy=msg->velocity.y;
     Vx=(int)(Vx*100)/100.0;
     Vy=(int)(Vy*100)/100.0;
     V=sqrt(pow(Vx,2)+pow(Vy,2));

     vel_cmd.linear.x=V;//载重的速度补偿
     vel_cmd.linear.y=0;
     vel_cmd.linear.z=0;

     vel_cmd.angular.x=0;
     vel_cmd.angular.y=0;
     vel_cmd.angular.z=msg->yaw_dot;

     vel_pub.publish(vel_cmd);

     ROS_INFO("Vx:%.2f ,Vy:%.2f ,V:%.2f ,yaw_dot:%.2f",Vx,Vy,V,vel_cmd.angular.z);
}
int main(int argc, char *argv[])
{
    int i=0,t=0;

    ros::init(argc, argv, "vel_ctrl");  //对该节点进行初始化操作
    ros::NodeHandle n;        
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Subscriber cmd_sub = n.subscribe("/planning/pos_cmd", 50, cmdCallback);//接收到目标点的信息后,对目标标志位置1,开始给小车速度指令
    ros::Rate loop_rate(50);//以50hz的速率发送
    while(ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
   }
       return 0;
}
