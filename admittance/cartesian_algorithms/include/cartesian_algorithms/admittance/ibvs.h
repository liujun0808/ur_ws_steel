
#ifndef IBVS_H
#define IBVS_H
// #ifndef VISP_HAVE_REALSENSE2
//   #if defined(VISP_CXX_STANDARD) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)

#include <iostream>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
// #include <visp3/robot/vpRobotUniversalRobots.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"


class ibvs
{
private:

    unsigned int width = 640, height = 480;  
    vpPoseVector ePc; // xyz rx ry rz
    vpHomogeneousMatrix eMc; // 转为齐次矩阵
    // Get camera intrinsics 
    vpCameraParameters cam;
    vpImage<unsigned char> I;
    vpHomogeneousMatrix cdMc, cMo, oMo; // 相机期望位姿，当前物体到相机的旋转矩阵，物体的位姿？
    vpHomogeneousMatrix cdMo; // 期望物体到相机的旋转矩阵
    vpServo task; // 视觉伺服任务
    std::vector<vpFeaturePoint> p,pd; // 定义特征点、期望特征点
    vpColVector v_c; // 速度控制向量
    std::vector<vpImagePoint>  corners_pd; // 期望特征角点像素坐标
    std::vector<vpImagePoint> corners_p;    // 实时特征角点像素坐标

    bool final_quit = false;  // 定义标志位
    bool send_velocities = false;
    bool servo_started = false;
    std::vector<vpPoint> point;



public:
    ibvs(/* args */);
    ~ibvs();
    double error = 20;
    bool has_converged = false;
    Eigen::MatrixXd jacobian;
    void conmupt_jacobi(const std::vector<double>& joint_pos);
    //主运行函数 
    geometry_msgs::TwistStamped ibvs_run(const std::vector<double> & ibvs_data,
        const std::vector<vpImagePoint> &corners_pd_new,const double &depth_des,int index);

};
//   #endif
// #endif



#endif
