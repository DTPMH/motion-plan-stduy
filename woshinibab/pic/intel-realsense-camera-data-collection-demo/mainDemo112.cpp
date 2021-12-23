/**********************************************************************//**
		Demo about data acquisition using class RealSenseData

@file		mainDemo.cpp
@author		WD
@date		2020
@brief		Data acquisition using class RealSenseData
**************************************************************************/
// class RealSenseData
#include "realSenseData.h"
// C++
#include <iostream>
#include <sstream>
#include <string>
// OpenCV
#include <opencv2/opencv.hpp>
// PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

using namespace std;
using namespace cv;
// 自定义函数
// convert data of cv::Mat format to PCL pointCloud
bool convertMatToPcl(const cv::Mat& xyzDepth, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
// combine XYZ pointCloud with Texture image, generate xyzrgb cloud
bool combineXyzWithTexture(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& xyzCloud, 
      const cv::Mat& textureImage, 
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& xyzRgbCloud
);


// demo
int main(int argc, char* argv[]) try
{
	const std::string pathOutput("/home/dtpmh/woshinibab/pic/intel-realsense-camera-data-collection-demo/buil/Test/");// 注意需要先自行创建文件夹
	// ---------------------------------------------------------------------------------------------------------
	
	string out_path="/home/dtpmh/woshinibab/pic/intel-realsense-camera-data-collection-demo/buil/video/test.map4";
	cv::VideoWriter video(out_path,CV_FOURCC('X','V','I','D'),25,Size(640,480));

	/*
			实例化类，进行数据采集实验
	*/
	RealSenseData cameraDataCapture;

	if (false == cameraDataCapture.devStart())
	{
		std::cerr << "[Error] Failed in starting device." << std::endl;
		return EXIT_FAILURE;
	}
	std::cout << "***********************************************" << std::endl;
	std::cout << "Is RS2 device connected or not: " << cameraDataCapture.isDevConnected() << std::endl;
	std::cout << "Is RS2 device ready (can capture Data) or not: " << cameraDataCapture.isDevReady() << std::endl;

	// RealSense SDK中读取 color 即2D相机内参
	auto intrMat_2d = cameraDataCapture.getIntrinsics_2dCamera();// 2D相机-内参
	std::cout << "intrMat_2d: \n" << intrMat_2d << std::endl;
	auto discoef_2d = cameraDataCapture.getDistortion_2dCamera();// 2D相机-畸变参数
	std::cout << "discoef_2d: \n" << discoef_2d << std::endl;
	std::cout << "------" << std::endl;


	// RealSense SDK中读取 depth 即3D相机内参
	auto intrMat_3d = cameraDataCapture.getIntrinsics_3dCamera();// 3D相机-内参
	std::cout << "intrMat_3d: \n" << intrMat_3d << std::endl;
	auto discoef_3d  = cameraDataCapture.getDistortion_3dCamera();// 3D相机-畸变参数
	std::cout << "discoef_3d: \n" << discoef_3d << std::endl;
	std::cout << "------" << std::endl;


	// RealSense SDK中读取外参 Depth-to-Color Extrinsics
	auto rotationMat = cameraDataCapture.getRotation();// 外参-旋转矩阵
	std::cout << "rotation: \n" << rotationMat << std::endl;
	auto translationMat  = cameraDataCapture.getTranslation();// 外参-平移向量
	std::cout << "translation: \n" << translationMat << std::endl;
	std::cout << "------" << std::endl;

	//
	// 循环获取数据并显示
	//
	int itest = 0;
	int pmh=0;
	std::string pmhnidaye;
	std::ostringstream temp0;
	temp0<<pmh;
	pmhnidaye=temp0.str();
	while (true) // 一直循环，没有退出条件
	{
		/*
			2D彩色图像
		*/
		cv::Mat colorImage_2dCamera = cameraDataCapture.getColorImage_2dCamera();
		if (0 == itest)
		{
			std::cout << "colorImage_2dCamera: \n"
				<< " - type: " << colorImage_2dCamera.type() << "\n"
				<< " - channel: " << colorImage_2dCamera.channels() << "\n"
				<< " - size: " << colorImage_2dCamera.size
				<< std::endl;
		}
		if (!colorImage_2dCamera.empty())
		{
			cv::imshow("Rgb", colorImage_2dCamera);
			std::string pathColor = pathOutput + "colorImage"+pmhnidaye+".png";
		    cv::imwrite(pathColor, colorImage_2dCamera);
		    std::cout << "Done: save colorImage_2dCamera as png-file." << std::endl;
		    pmh++;
		    std::ostringstream temp;
		    temp<<pmh;
	        pmhnidaye=temp.str();
	        //video.write(colorImage_2dCamera);

		}
		// 保存 2D彩色图像 到本地，见后


		// std::cout << "Running " << itest << " ..." << std::endl;
		itest++;
	}
	// ---------------------------------------------------------------------------------------------------------
	/*
	 		关闭设备
	*/
	if (false == cameraDataCapture.devStop()) // TODO. 目前返回值一定是true，内部未实现
	{
		std::cerr << "[Error] Stop device failed." << std::endl;
		return EXIT_FAILURE;
	}
	else
	{
		std::cout << "Device stopped. " << std::endl;
	}
	std::cout << "----------------------------------" << std::endl;
	std::cout << "------------- closed -------------" << std::endl;
	std::cout << "----------------------------------" << std::endl;
	
	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}


// convert cv::Mat format to pcl::PointCloud<pcl::PointXYZ>::Ptr
bool convertMatToPcl(const cv::Mat& xyzDepth, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // xyzDepth 的数据类型必须是 CV_32FC3
    if(21 != xyzDepth.type())
    {
        std::cerr << "Error in convertMatToPcl()." << std::endl;
		std::cerr << "The type of input data(cv::Mat) must be CV_32FC3." << std::endl;
        return false;
    }
	// cv::Mat数据大小
	auto depthWidth = xyzDepth.cols;
	auto depthHeight = xyzDepth.rows;

	cloud->width = static_cast<uint32_t>(depthWidth);
	cloud->height = static_cast<uint32_t>(depthHeight);
	cloud->is_dense = false;
	cloud->points.resize(cloud->height * cloud->width);

	pcl::PointXYZ* pt = &cloud->points[0];
	for (int iRows = 0; iRows < depthHeight; iRows++)
	{
		for (int iCols = 0; iCols < depthWidth; iCols++, pt++)
		{
			pt->x = xyzDepth.at<cv::Vec3f>(iRows, iCols)[0];
			pt->y = xyzDepth.at<cv::Vec3f>(iRows, iCols)[1];
			pt->z = xyzDepth.at<cv::Vec3f>(iRows, iCols)[2];
		}
	}
	return true;
}


// generate XYZRGB pointCloud by combining XYZ pointCloud with Texture image
//	[IN] pointCloud xyz information in PCL format
//	[IN] texture image (CV_8UC3)
//	[OUT] pointCloud xyzRgb information in PCL format
bool combineXyzWithTexture(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& xyzCloud, 
    const cv::Mat& textureImage, 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& xyzRgbCloud
)
{
    // textureImage 的数据类型必须是 CV_8UC3
    if(16 != textureImage.type())
    {
        std::cerr << "[Error] The type of textureImage(cv::Mat) must be CV_8UC3." << std::endl;
        return false;
    }

    xyzRgbCloud->width = xyzCloud->width;
    xyzRgbCloud->height = xyzCloud->height;
    xyzRgbCloud->is_dense = false;
    xyzRgbCloud->points.resize(xyzRgbCloud->height * xyzRgbCloud->width);

    pcl::PointXYZRGB* pt = &xyzRgbCloud->points[0];
    for (int i = 0; i < xyzCloud->width * xyzCloud->height; i++)
    {
        if (xyzCloud->points[i].z < 1e-5)// 无效点云用 0 (z=0) 表示的，新点云中改为 z=NaN 表示
        {
            pt->x = std::numeric_limits<float>::quiet_NaN();
            pt->y = std::numeric_limits<float>::quiet_NaN();
            pt->z = std::numeric_limits<float>::quiet_NaN();
        }
        else
        {
            pt->x = xyzCloud->points[i].x;
            pt->y = xyzCloud->points[i].y;
            pt->z = xyzCloud->points[i].z;
        }

        pt->b = textureImage.at<cv::Vec3b>(i)[0];// CV_8UC3, 三个通道依次表示 bgr
        pt->g = textureImage.at<cv::Vec3b>(i)[1];
        pt->r = textureImage.at<cv::Vec3b>(i)[2];

        pt++;
    }

    return true;
}
