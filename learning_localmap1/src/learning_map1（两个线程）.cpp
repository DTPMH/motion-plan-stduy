#include <ros/ros.h>//ros必须的头文件
#include <cmath>

#include <Eigen/Geometry>//用到Eigen的几何模块时需要包含的头文件,即变换矩阵的定义等
#include <Eigen/Dense>//用到一些常用的函数时，需要包含的头文件
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

//pcl
#include <sensor_msgs/PointCloud2.h>  //用到ROS中的点云信息的时候需要包含的头文件
#include <pcl_conversions/pcl_conversions.h>  //用到pcl与ROS点云转换的时候需要包含的头文件
#include <pcl/common/transforms.h>//坐标系变换的时候需要用到的头文件
#include <pcl/point_types.h> //用到定义pcl点的类型时需要包含的头文件
#include <pcl/filters/passthrough.h>//用到直通点云滤波时需要包含的头文件
#include <pcl/filters/statistical_outlier_removal.h>//用到高斯点云滤波的时候需要包含的头文件
#include <pcl/filters/voxel_grid.h>//用到体素滤波时需要包含的头文件
#include <pcl/point_cloud.h>//定义pcl中点云类型的时候需要包含的头文件

#include <thread>//用到线程函数时需要包含的头文件
#include <mutex>//用到线程函数以及互斥量，共同处理共享信息时需要包含的头文件


using namespace std;
using namespace Eigen;

nav_msgs::Odometry vision_odom;//用来接收传来的定位结果
sensor_msgs::PointCloud2 local_map;//用来发布转换到世界坐标系下的点云结果
sensor_msgs::PointCloud2 trans_clouds;//用来接受相机发布的点云结果
ros::Publisher pub_cloud;//定义发布话题的句柄
double time0,time1;//分别定义接收到定位结果与接受到相机点云的时间
ros::Time time2,time3;//定义当前时间
Eigen::Vector3d t;//定义相机坐标系相对世界坐标系原点的平移量
Eigen::AngleAxisd R0(M_PI,Eigen::Vector3d (1,0,0));//定义一个将相机坐标系与世界坐标系对准的旋转矩阵R0，表示顺着X轴旋转180度
std::mutex my_mutex; //定义共享信息互斥量

int get_vins=0,get_clouds=0;

/****************************定义用到的共享点云信息*******************************************************************************/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Camera_cloud_PCL(new pcl::PointCloud<pcl::PointXYZRGB>);//定义PCL类型的点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Camera_cloud_pass_filter(new pcl::PointCloud<pcl::PointXYZRGB>);//定义直通滤波后的点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Camera_cloud_statistical_filter(new pcl::PointCloud<pcl::PointXYZRGB>);//定义高斯滤波后的点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Camera_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);//定义滤波后的相机坐标系下的点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr World_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);//定义转换后的世界坐标系下的点云
/*****************************************************************************************************************************/



Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q)
{
    float quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}


void VINS_odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    //ROS_INFO("GET odom");
    get_vins=1;
    my_mutex.lock();
    vision_odom=*msg;
    my_mutex.unlock();

}
void points_camera_Callback(const sensor_msgs::PointCloud2& camera_pointclouds)
{
    //cout<<"get cloud"<<endl;
    get_clouds=1;
    my_mutex.lock();
    //string ss=camera_pointclouds.header.frame_id;
    trans_clouds=camera_pointclouds;
    my_mutex.unlock();
    //cout<<"frame:"<<ss<<endl;

}

void camerclouds_2_worldclouds()
{
    while(ros::ok())
    {
        //cout<<"come"<<endl;
        if (get_clouds!=1)
        {
            cout<<"没有接受到点云信息"<<endl;
        }
        else
        {
            time2=ros::Time::now();
            my_mutex.lock();
            time0=vision_odom.header.stamp.toSec();//记录收到定位结果时的时间

            //ROS_INFO("VISION");
            //ROS_INFO("x:%.2f ,y:%.2f ,z:%.2f ,Qx:%.2f ,Qy:%.2f ,Qz:%.2f,Qw:%.2f",vision_odom.pose.pose.position.x,vision_odom.pose.pose.position.y,vision_odom.pose.pose.position.z,
            //vision_odom.pose.pose.orientation.x,vision_odom.pose.pose.orientation.y,vision_odom.pose.pose.orientation.z,vision_odom.pose.pose.orientation.w);

            my_mutex.unlock();
            double dt=time1-time0;
            cout<<"time0:"<<time0<<endl;
            cout<<"time1:"<<time1<<endl;
            cout<<"时间差："<<dt<<endl;
            if(fabs(time1-time0)>0.2)
            {
                cout<<"定位结果与点云不对齐"<<endl;//如果点云与定位结果的时间差在0.2s以内，则认为该配对生效，（因为点云发布的速度是10hz,而定位结果发布的速度是5hz)待定
            }
            else
            {
                my_mutex.lock();
        /***********************************由定位结果计算转换矩阵***************************************************************/ 
                t<<vision_odom.pose.pose.position.x,vision_odom.pose.pose.position.y,vision_odom.pose.pose.position.z;//定义平移向量
                Eigen::Quaterniond q(vision_odom.pose.pose.orientation.w,vision_odom.pose.pose.orientation.x,vision_odom.pose.pose.orientation.y,vision_odom.pose.pose.orientation.z);//定义旋转四元数
                
                my_mutex.unlock();
                
                Eigen::Matrix3d R;
                Eigen::Vector4d t0;
                t0<<0,0,0,1;
                //q=q.normalized();
                //Eigen::Vector3d eulerAngle=quaternion_to_euler(q);
                //cout<<"滚转角roll："<<eulerAngle(0)*180/M_PI<<"俯仰角pitch："<<eulerAngle(1)*180/M_PI<<"偏航角yaw："<<eulerAngle(2)*180/M_PI<<endl;
                R=q.toRotationMatrix();//由四元数转换为旋转矩阵
                R=R*R0;//将相机坐标系与世界坐标系对准是的旋转加入到转换矩阵中
                Eigen::Matrix<double,4,4> T;//生成4*4的变换矩阵
                T.block(0,0,3,3)=R;
                T.block(0,3,3,1)=t;
                T.block(3,0,1,4)=t0.transpose();
                //T=T.inverse().eval();
                /*******************************************************************************************************************/
                
                pcl::transformPointCloud(*Camera_cloud,*World_cloud,T);
                
                //将其转换为ROS下的点云类型，并将其以话题的形式发布
                
                World_cloud->width = World_cloud->points.size();
                World_cloud->height = 1;
                World_cloud->is_dense = true;
                cout<<"世界坐标系下的点云数量："<<World_cloud->points.size()<<endl;
                
                pcl::toROSMsg(*World_cloud, local_map);
                
                local_map.header.frame_id = "world";

                pub_cloud.publish(local_map);

                //cout<<"点云已发布"<<endl;
                time3=ros::Time::now();
               
                double cout_time=(time3-time2).toSec();
                
                cout<<"处理信息转换总共花费的时间:"<<cout_time<<endl;
            }
        }
    }
}
void cameraclouds_filter()
{
    cout<<"come"<<endl;
    time1=trans_clouds.header.stamp.toSec();//记录收到点云信息的时间
    pcl::fromROSMsg(trans_clouds, *Camera_cloud_PCL);//将接收到的ROS类型的点云信息转换成PCL中的类型，以便调用PCL库进行处理
    cout<<"收到的点云数量："<<Camera_cloud_PCL->points.size()<<endl;
    /******************************************对接收到的点云进行滤波以及坐标转换处理*****************************************/             
                
    //直通深度滤波
    pcl::PassThrough<pcl::PointXYZRGB> pass_filter;
    pass_filter.setInputCloud(Camera_cloud_PCL);
    pass_filter.setFilterFieldName("z");
    pass_filter.setFilterLimits(0.01,5.0);
    //pass.setFilterLimitsNegative(true);
    pass_filter.filter(*Camera_cloud_pass_filter);
    cout<<"直通滤波后的点云数量："<<Camera_cloud_pass_filter->points.size()<<endl;
                
    //均值滤波
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;//定义高斯滤波器，除去离群点，即均值滤波
    statistical_filter.setMeanK(50);//对每个点分析的邻近点个数设为50
    statistical_filter.setStddevMulThresh(1.0);//标准差倍数设为1
    statistical_filter.setInputCloud(Camera_cloud_pass_filter);//输入点云
    statistical_filter.filter(*Camera_cloud_statistical_filter);//tmp表示滤波之后的点云
    cout<<"均值滤波后的点云数量："<<Camera_cloud_statistical_filter->points.size()<<endl;
                
    //体素滤波
    pcl::VoxelGrid<pcl::PointXYZRGB>voxel_filter;
    voxel_filter.setLeafSize(0.01f, 0.01f, 0.01f);//对转换过来的点云进行体素滤波
    voxel_filter.setInputCloud(Camera_cloud_statistical_filter);
    voxel_filter.filter(*Camera_cloud);
    cout<<"体素滤波后的点云数量："<<Camera_cloud->points.size()<<endl;


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "make_LocalMap");
    ros::NodeHandle n;   

    ros::Subscriber vision_sub = n.subscribe("/vins_estimator/odometry", 1000, VINS_odomCallback);
    //ros::Subscriber left_image_sub = n.subscribe("/mynteye/left/image_color",1,left_image_Callback);
    //ros::Subscriber depth_image_sub = n.subscribe("/mynteye/depth/image_raw",1,depth_image_Callback);
    ros::Subscriber points_camera_sub = n.subscribe("/mynteye/points/data_raw",1,points_camera_Callback);
    pub_cloud = n.advertise<sensor_msgs::PointCloud2>("/local_map/cloud", 10);
    //ros::Rate rate(10);//50-100hz即可
    std::thread t1(camerclouds_2_worldclouds);
    std::thread t2(cameraclouds_filter);
    t1.join();
    t2.join();
  /*  
    while(ros::ok())
    {
        ros::spinOnce();
        t1.join();
        rate.sleep();
    }
    */
    ros::spin();
    return 0;
}