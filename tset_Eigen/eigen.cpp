#include <iostream>
#include <Eigen/Geometry>//用到Eigen的几何模块时需要包含的头文件,即变换矩阵的定义等
#include <Eigen/Dense>//用到一些常用的函数时，需要包含的头文件
using namespace std;
int main(int argc, char const *argv[])
{
    Eigen::Vector3d t;
    t<<0,0,1;//定义平移向量
    Eigen::Quaterniond q(0.1,0.35,0.2,0.3);//定义旋转四元数w,x,y,z
    Eigen::Matrix3d R;
    Eigen::Vector4d t0;
    t0<<0,0,0,1;
    cout<<"q0:\n"<<q.coeffs()<<endl;
    q=q.normalized();
    cout<<"q1:\n"<<q.coeffs()<<endl;
    R=q.toRotationMatrix();
    Eigen::Matrix<double,4,4> T;//生成4*4的变换矩阵
    T.block(0,0,3,3)=R;
    T.block(0,3,3,1)=t;
    T.block(3,0,1,4)=t0.transpose();
    cout<<"变换矩阵T:\n"<<T<<endl;
    return 0;
}

