#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include "ur10_start/cartesianPath.h"
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <python3.8/Python.h>
#include <Eigen/Core>



void gripper_connect()
{
    PyRun_SimpleString("from jodellSdk.jodellSdkDemo import ClawEpgTool");
    PyRun_SimpleString("clawTool = ClawEpgTool()");
    PyRun_SimpleString("comlist = clawTool.searchCom()");
    PyRun_SimpleString("flag_connect = clawTool.serialOperation('/dev/ttyUSB0', 115200, True)");
    PyRun_SimpleString("\nprint(flag_connect)\nflag_enable = clawTool.clawEnable(9, True)");
}
void gripper_close()
{
    PyRun_SimpleString("clawTool.runWithoutParam(9, 2)");
}
void gripper_open()
{
    PyRun_SimpleString("clawTool.runWithoutParam(9, 1)");
}

int main(int argc, char** argv)
{   
    //设置ros输出中文
    setlocale(LC_ALL,"");

    // 夹爪使能,并打开夹爪
    Py_Initialize();//Python解释器初始化
    // gripper_connect();
    // gripper_open();
    ros::WallDuration(3).sleep();

    //ros节点初始化
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //定义坐标监听器
    tf2_ros::Buffer tfBuffer;//tf缓冲区
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::WallDuration(3).sleep();

    //创建笛卡尔规划对象，进行机械臂位姿初始化
    cartesianPath cPath_test;
    cPath_test.arm.setNamedTarget("home");
    cPath_test.arm.move();
    ROS_INFO("wait 8s.......");
    ros::WallDuration(8).sleep();

    //创建变换矩阵
    geometry_msgs::TransformStamped transformStamped,transformStamped2;
    try
    {
        transformStamped = tfBuffer.lookupTransform("base", "tool0", ros::Time(0));
    }
    catch(tf2::TransformException& ex)
    {
        std::cerr << ex.what() << '\n';
    }
    
    // try
    // {
    //     transformStamped2 = tfBuffer.lookupTransform("camera_color_frame", "object_10", ros::Time(0));
    // }
    // catch(tf2::TransformException& ex)
    // {
    //     std::cerr << ex.what() << '\n';
    // }

    // if (transformStamped2.transform.translation.z == 0 || transformStamped2.transform.translation.z > 0.7)
    // {
    //     ROS_INFO("相机识别有误");
    //     Py_Finalize();
    //     ros::shutdown();
    //     return 0;
    // }
    
    Eigen::Matrix4Xf object_pos_c(4,1);//点在相机坐标系下的值,齐次化
    object_pos_c(0,0) = transformStamped2.transform.translation.y * -1;
    object_pos_c(1,0) = transformStamped2.transform.translation.z * -1;
    object_pos_c(2,0) = transformStamped2.transform.translation.x;
    object_pos_c(3,0) = 1;


    Eigen::Matrix3f tool02base;//tool0到base的旋转矩阵
    Eigen::Quaternionf q;//临时变量
    q.x() = transformStamped.transform.rotation.x;
    q.y() = transformStamped.transform.rotation.y;
    q.z() = transformStamped.transform.rotation.z;
    q.w() = transformStamped.transform.rotation.w;
    tool02base =  q.toRotationMatrix();//四元素转旋转矩阵
    Eigen::Matrix4f tool02base_;//tool0到base的旋转矩阵齐次化
    tool02base_<<tool02base(0,0),tool02base(0,1),tool02base(0,2),transformStamped.transform.translation.x,
                tool02base(1,0),tool02base(1,1),tool02base(1,2),transformStamped.transform.translation.y,
                tool02base(2,0),tool02base(2,1),tool02base(2,2),transformStamped.transform.translation.z,
                0,0,0,1;
    std::cout<<tool02base_<<std::endl;

    Eigen::Matrix4f camera2tool0_;//外参矩阵，camera到tool0
    camera2tool0_<<0.99921847,0.02002815,-0.03407832,-0.01114759,
                    -0.01984378 ,0.99978662,0.0057399 ,-0.10696188,
                    0.03418601,-0.00505917,0.99940268,-0.13623781,
                    0,0,0,1;

    Eigen::Matrix4Xf object_pos_b(4,1);//点在base坐标系下的值
    Eigen::Matrix4Xf object_pos_t(4,1);//点在tool0坐标系下的值
    // Eigen::Matrix4Xf object_pos_t_inv(4,1);//点在tool0坐标系下的值,外参矩阵求逆后
    // object_pos_t_inv = camera2tool0_.inverse().eval() * object_pos_c ;
    object_pos_t = camera2tool0_ * object_pos_c ;
    object_pos_b = tool02base_ * object_pos_t;

    std::cout<<"点在相机坐标系下\n"<<object_pos_c.transpose()<<std::endl;
    std::cout<<"点在tool0坐标系下\n"<<object_pos_t.transpose()<<std::endl;
    // std::cout<<"外参求逆后点在tool0坐标系下\n"<<object_pos_t_inv.transpose()<<std::endl;
    std::cout<<"点在base坐标系下\n"<<object_pos_b.transpose()<<std::endl;


    geometry_msgs::Pose curPose;//当前末端在base的位姿
    //转为base坐标系
    curPose.orientation = transformStamped.transform.rotation;
    curPose.position.x   = transformStamped.transform.translation.x;
    curPose.position.y   = transformStamped.transform.translation.y;
    curPose.position.z   = transformStamped.transform.translation.z;
    std::cout<<curPose<<std::endl;

    std::vector<geometry_msgs::Pose> waypoits;
    //pick-------
    curPose.position.z =object_pos_b(2,0)+0.032;
    waypoits.push_back(curPose);

    curPose.position.y =object_pos_b(1,0);
    waypoits.push_back(curPose);
    geometry_msgs::Pose  pick_pose = curPose ;

    cPath_test.doCartesianPath(waypoits);
    waypoits.clear();
    gripper_close();
    ros::WallDuration(2).sleep();

    //place-------
    curPose.position.y -= 0.3;
    waypoits.push_back(curPose);

    curPose.position.z -= 0.2;
    waypoits.push_back(curPose);

    curPose.position.y = pick_pose.position.y;
    waypoits.push_back(curPose);

    cPath_test.doCartesianPath(waypoits);
    waypoits.clear();
    gripper_open();
    ros::WallDuration(2).sleep();

    //up
    curPose.position.y -= 0.3;
    waypoits.push_back(curPose);
    cPath_test.doCartesianPath(waypoits);
    waypoits.clear();
    ros::WallDuration(1).sleep();

    //move to home
    cPath_test.arm.setNamedTarget("home");
    cPath_test.arm.move();
    std::cout<<"end"<<std::endl;
    ros::shutdown();
    Py_Finalize();//关闭解释器
    return 0;
}


    // system("python gripper_open.py");


    // ros::Rate rate(10.0);
    // while (ros::ok())
    // {
    //     geometry_msgs::TransformStamped transformStamped;
    //     try
    //     {
    //         // 获取位姿信息
    //         // double x = transformStamped.transform.translation.x;
    //         // double y = transformStamped.transform.translation.y;
    //         double x = transformStamped.transform.translation.x;
    //         double y = transformStamped.transform.translation.y;
    //         double z = transformStamped.transform.translation.z;
    //         double qx = transformStamped.transform.rotation.x;
    //         double qy = transformStamped.transform.rotation.y;
    //         double qz = transformStamped.transform.rotation.z;
    //         double qw = transformStamped.transform.rotation.w;

    //         // 将四元数转换为RPY角
    //         tf2::Quaternion quat(qx, qy, qz, qw);
    //         tf2::Matrix3x3 mat(quat);
    //         double roll, pitch, yaw;
    //         mat.getRPY(roll, pitch, yaw);

    //         // 打印位姿信息
    //         ROS_INFO("End Effector Pose:");
    //         ROS_INFO("Position: x=%.3f, y=%.3f, z=%.3f", x, y, z);
    //         // ROS_INFO("Orientation: roll=%.3f, pitch=%.3f, yaw=%.3f", roll, pitch, yaw-3.1415926);
    //         ROS_INFO("Orientation: roll=%.3f, pitch=%.3f, yaw=%.3f", roll, pitch, yaw);

    //     }
    //     catch (tf2::TransformException& ex)
    //     {
    //         ROS_WARN("%s", ex.what());
    //     }

    //     rate.sleep();
    // }