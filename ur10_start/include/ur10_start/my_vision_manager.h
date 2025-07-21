#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include "std_msgs/Float32MultiArray.h"
// #include <qt5/QtGui/qtransform.h>
// #include <QTransform>

class my_vision_manager
{
private:
    
    void detect2DObject(cv::Mat msg,float &x,float &y);

    // void detecTable(cv::Rect &tablePose);


    float curr_pixel_centre_x,curr_pixel_centre_y,curr_pixel_centre_z;//对齐后深度图中的像素值
    float temp_z;//未解决深度图丢帧而设置的临时变量

    // cv::Mat  curr_img;
    // cv::Mat  curr_img_d;

    cv_bridge::CvImagePtr cv_ptr;//cv bridge中的图像指针
    cv_bridge::CvImagePtr cv_ptr_d;

    Eigen::Matrix3d camera_k;//内参矩阵

    Eigen::Matrix4d camera2tool0;//外参矩阵
    
    Eigen::Matrix3d color2depth;//彩色到深度对齐的旋转矩阵
    Eigen::Matrix4d color2depth_;//彩色到深度对齐的旋转齐次矩阵



    image_transport::ImageTransport it_;

    image_transport::Subscriber image_color,image_depth;//subscribe first image
    // image_transport::Publisher  image1_pub;//publish table
    // image_transport::Publisher  image2_pub;//publish object
    // void get2DLocation(cv::Mat msg,float &x,float &y);

    void detectDepth(cv::Mat msg_d);

    Eigen::MatrixXd pixel_location;   // 抓取点在像素坐标系
    Eigen::MatrixXd camera_location;  // 抓取点在相机坐标系
    Eigen::Matrix3d camera_end_rMat;//相机到末端的四元数转换的旋转矩阵


    void convertToMM();
    // **********
    cv::Point center;//物体中点像素
public:
    my_vision_manager(ros::NodeHandle n_);

    // void get3DLocation(float &x,float &y,float &z);
    // Eigen::Matrix<float,4,1> point2Cam;
    // float x,y,z;//点在相机坐标系下的值
    Eigen::MatrixXd point2cam_4;  // 抓取点在相机坐标系齐次化
    Eigen::MatrixXd point2tool_4;  // 抓取点在末端坐标系齐次化
    void imageCb(const sensor_msgs::ImageConstPtr &msg);
    void imageCb_d(const sensor_msgs::ImageConstPtr &msg_d);
    void objectsDetectedCb(const std_msgs::Float32MultiArrayConstPtr & msg);
    void grasp();

};


