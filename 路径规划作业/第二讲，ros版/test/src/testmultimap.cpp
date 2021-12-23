#include <ros/ros.h>
#include <iostream>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <fstream>
#include <math.h>
#include<map>
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh("~");
    std::multimap<double,char> test1;
    std::multimap<double,char>::iterator l;
    test1.insert(std::make_pair(1,'a'));
    test1.insert(std::make_pair(2,'b'));
    l=test1.begin();
    test1.erase(l);
    l=test1.begin();
    std::cout<<l->second<<std::endl;

    return 0;
}