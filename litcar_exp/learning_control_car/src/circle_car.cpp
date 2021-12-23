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

 nav_msgs::Odometry pot_really;//odom点信息
 geometry_msgs::PoseStamped pot_goal,finally_goal;//引路点以及最终目标点信息
 geometry_msgs::Twist vel_cmd; //声明一个geometry_msgs::Twist 类型的对象vel_cmd，并将速度的值赋值到这个对象里面
 ros::Publisher vel_pub;//定义速度发布
 int target=0,my_cout=0,have_odom=0,end_flag=0;
 float delat_x_path,delat_y_path,d;//接受引路点的判断

 Eigen::Vector3f pt=Eigen::Vector3f::Zero();//
 Eigen::Vector3f ps;
 vector<Eigen::Vector3f> pos_pts;

 void pose_cmdcallback(const geometry_msgs::PoseStamped::ConstPtr& msg)//接受引路点,当上一个点与现在的点相同时,停止接受.
{
  delat_x_path=pt(0)-msg->pose.position.x;
  delat_y_path=pt(1)-msg->pose.position.y;
  delat_x_path=(int)(delat_x_path*100)/100.0;
  delat_y_path=(int)(delat_y_path*100)/100.0;
  d=sqrt(pow(delat_x_path,2)+pow(delat_y_path,2));//d表示上一个位置点与现在接收到的位置点的距离
  if (d!=0)
  {
     pt(0)=msg->pose.position.x;
     pt(1)=msg->pose.position.y;
     pt(2)=msg->pose.position.z;//将位置点赋值给三维向量点pt
     pos_pts.push_back(pt);     //将位置点放入引路点矩阵中
     //ROS_INFO("come in ");
     /*ROS_INFO("x: %.2f ,y: %.2f ,z: %.2f ",
              pt(0),pt(1),pt(2));*/
  }
  else
  {
    // ROS_INFO("stop ");
  }
}
void odometryCallback(const nav_msgs::OdometryConstPtr& msg) {//接受odom点
  pot_really.pose.pose.position.x=msg->pose.pose.position.x;
  pot_really.pose.pose.position.y=msg->pose.pose.position.y;
  pot_really.pose.pose.position.z=msg->pose.pose.position.z;
  pot_really.pose.pose.orientation.x=msg->pose.pose.orientation.x;
  pot_really.pose.pose.orientation.y=msg->pose.pose.orientation.y;
  pot_really.pose.pose.orientation.z=msg->pose.pose.orientation.z;
  pot_really.pose.pose.orientation.w=msg->pose.pose.orientation.w;  
}
void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {//接受目标点,并将target==1,并将end_flag==0;
  if (msg->pose.position.z < -0.1) return;
  finally_goal.pose.position.x=msg->pose.position.x;
  finally_goal.pose.position.y=msg->pose.position.y;
  finally_goal.pose.position.z=msg->pose.position.z;;
  target = 1;
  end_flag=0;//end_flag表示当前的引路点是最后一个引路点
  ROS_INFO("receive target ");
}
int main(int argc, char *argv[])
{
    float omega,omega1,omega2,omega3=0,e,last_e=0.0,last_last_e=0.0;//定义角度误差等数值
    int i=0,t=0;
    //omega表示小车在当前的姿态上转到目标点需要的角度,Omega1表示目标点与当前点之间连线的角度,omega2表示小车当前的偏航角,omega3表示的是给定小车的角度度指令
    //e表示横轴误差(V*sin(omega)),last_e表示上次的误差,last_last_e表示上上次的误差,用于PID控制公式

    float V,deltaX=0,deltaY=0,deltaOmega=0,distance=0,delta_X=0,delta_Y=0;//定义两点之间的距离等数值
    //V表示引路点与当前点之间的距离,deltaX,Y表示引路点与当前点之间X,Y方向之间的偏差,用来计算V,以及Omega1;distance表示当前点与最终点之间的距离
    //delta_X表示的是当前点与最终目标点之间的X,Y方向之间的距离,deltaomega表示的是小车角速度的偏差值(利用PID计算得出)

    float Kp=0.65,Ki=0.00,Kd=0.0;//PID参数

    ros::init(argc, argv, "vel_ctrl");  //对该节点进行初始化操作
    ros::NodeHandle n;         //申明一个NodeHandle对象n，并用n生成一个广播对象vel_pub
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);////vel_pub会在主题"/cmd_vel"(机器人速度控制主题)里广播geometry_msgs::Twist类型的数据
    
    ros::Subscriber pose_cmd_sub = n.subscribe("/my_position_cmd", 10, pose_cmdcallback);//接收到路径点进行回调函数,赋值引路点
    ros::Subscriber odom_sub = n.subscribe("/state_ukf/odom", 1, odometryCallback);//接收到小车的定位信息后,调用函数,将小车的姿态传入全局变量中
    ros::Subscriber way_sub = n.subscribe("/my_waypoint", 1, waypointCallback);//接收到目标点的信息后,对目标标志位置1,开始给小车速度指令
    
    ros::Rate loop_rate(10);//以10hz的速率发送

    while(ros::ok())
    {   

     if (target==0)//如果没有设定目标点,则将其小车速度设置为0
     {
          pos_pts.clear();//如果到达目标,则将引路点矩阵清除
          i=1;

          vel_cmd.linear.x = 0.0;
          vel_cmd.linear.y = 0.0; 
          vel_cmd.linear.z = 0.0;

          vel_cmd.angular.x = 0;
          vel_cmd.angular.y = 0;
          vel_cmd.angular.z = 0;

          vel_pub.publish(vel_cmd); //赋值完毕后，发送到主题"/cmd_vel"

          ROS_INFO("no target ");
     }
     if (target==1)//如果目标点已经给定,则开始对小车提供速度指令
        {
          /*****************************以下代码为了判断是否达到目标************************************************/
          delta_X=finally_goal.pose.position.x-pot_really.pose.pose.position.x;
          delta_Y=finally_goal.pose.position.y-pot_really.pose.pose.position.y;
          delta_X=(int)(delta_X*100)/100.0;
          delta_Y=(int)(delta_Y*100)/100.0;
          distance=sqrt(pow(delta_X,2)+pow(delta_Y,2));
          ROS_INFO("distance: ( %.2f) ",distance);//将当前点与目标点之间的距离打印出来

          if (fabs(distance)<=0.2)//计算odom点与目标点之间的距离.若是超过判断是否到达目标点的阈值,则判断到达目标.令target=0
             {
                target=0;
                ROS_INFO("IM arrive");
                pos_pts.clear();//如果到达目标,则将引路点矩阵清除
                i=1;//并将引路点索引设为1
             }
           /****************************************************************************************************/
          if ((pos_pts.size()>=50)&&target==1)//若是引路点的数量大于50,并且target==1,则开始控制小车
          {
              pot_goal.pose.position.x=pos_pts[i](0);
              pot_goal.pose.position.y=pos_pts[i](1);
              pot_goal.pose.position.z=pos_pts[i](2);//为引路点赋值
            
            
              deltaX=pot_goal.pose.position.x-pot_really.pose.pose.position.x;
              deltaY=pot_goal.pose.position.y-pot_really.pose.pose.position.y;
              deltaX=(int)(deltaX*100)/100.0;
              deltaY=(int)(deltaY*100)/100.0;//计算引路点与odom点的在X.Y轴间的差值

              ROS_INFO("pot_goal: (x %.3f, y %.3f. z %.3f) ",
                        pos_pts[i](0),
                        pos_pts[i](1),
                        pos_pts[i](2));//将引路点打印出来

              ROS_INFO("pot_really: (x %.3f, y %.3f. z %.3f) ",
                  pot_really.pose.pose.position.x,
                  pot_really.pose.pose.position.y,
                  pot_really.pose.pose.position.z);//将odom点打印出来

              V=sqrt(pow(deltaX,2)+pow(deltaY,2));//计算出odom点与当前引路点之间的距离,
                                                  //可以用来表示小车线速度的误差(对其进行PID参数的误差)
              ROS_INFO("V: ( %.2f) ",V);//将当前点与引路点之间的距离打印出来

              if (((V<=1.0)||(V>5.0))&&end_flag==0)//如果引路点与odom点之间的距离小于阈值,或者大于错误阈值,则赋值之后的第5个引路点
             {
                  i+=5;
               if (i>=pos_pts.size())//如果引路点的索引大于引路点数组的长度,则令索引等于最后一个引路点,并且将end_flag=1,说明已经是最后一个引路点了;
               {
                   i=pos_pts.size()-1;
                   end_flag=1;
                   ROS_INFO("end");
               }
               
               ROS_INFO("qianjin");
             }
             else//若是引路点与odom之间的距离大于阈值,则开始计算小车的速度指令.
             {
               omega1=atan2(deltaY,deltaX);//当前点与路径点的角度
               omega2=atan2(
                       2*(pot_really.pose.pose.orientation.w*pot_really.pose.pose.orientation.z+pot_really.pose.pose.orientation.x*pot_really.pose.pose.orientation.y),
                       1-2*(pow(pot_really.pose.pose.orientation.y,2)+pow(pot_really.pose.pose.orientation.z,2)));//计算出小车目前的姿态
               omega=omega1-omega2;//(若是W为负的.则向右拐,若是W为正的,则向左拐)(小车角速度正的向左拐)
               e=V*sin(omega);//e表示的就是控制误差
               deltaOmega=Kp*(e-last_e)+Ki*e+Kd*(e-2*last_e+last_last_e);
               omega3+=deltaOmega;
               //计算出来的小车的角速度(其实就是小车的偏航角减去当前点与路径点的角度),若是值大于0,则表明需要向右旋转一个角度,若是值小于0,则表示需要   向左旋转一个角度
               if(omega3>M_PI)
               omega3=M_PI;
               if(omega3<-M_PI)
               omega3=-M_PI; //设置其最大值
             
               ROS_INFO("err: (x %.2f, y %.2f. deltaOmega %.2f,omega1 %.2f,omega2 %.2f,omega %.3f,omega3 %.3f) ",
                         deltaX,
                         deltaY,
                         deltaOmega,
                         omega1,omega2,omega,omega3);//将当前点与路径点之间的差,以及小车的角度,以及计算出来的旋转角度表示出来
               last_last_e=last_e;//PID
               last_e=e;//留着进行PID改进

               t++;
               if (t>=2)
               {
                 vel_cmd.linear.x = 0.2;
                 vel_cmd.linear.y = 0.0; 
                 vel_cmd.linear.z = 0.0;

                 vel_cmd.angular.x = 0;
                 vel_cmd.angular.y = 0;
                 vel_cmd.angular.z = omega3;

                 vel_pub.publish(vel_cmd); //赋值完毕后，发送到主题"/cmd_vel"
               }
             }//计算小车速度指令结束
          }//控制小车指令结束
        }//target==1指令结束
        ros::spinOnce();
        loop_rate.sleep();
    }//while循环结束
    return 0;
}
