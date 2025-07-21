#include "ros/ros.h"
#include "ur10_start/cartesianPath.h"
#include "moveit/move_group_interface/move_group_interface.h"//C++与move_group的接口
#include "moveit/robot_trajectory/robot_trajectory.h"
#include <tf/transform_listener.h>
// #include <>

void printCurrentPose(geometry_msgs::Pose currentPose)
{
    ROS_INFO("当前位置：%0.3f,%0.3f,%0.3f", currentPose.position.x,
             currentPose.position.y, currentPose.position.z-1.03);
    ROS_INFO("当前姿态四元数：%0.3f,%0.3f,%0.3f,%0.3f", currentPose.orientation.w,
             currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z);
}

// tf::Vector3 getPosition()
// {   
//     // define listener
//     tf::TransformListener tf_box_to_robot;
    
//     //define transform matrix
//     tf::StampedTransform box_to_robot; 

//     tf::Vector3 box_world_fram,box_robot_fram;

//     box_world_fram.setX(0.689);
//     box_world_fram.setY(0.614);
//     box_world_fram.setZ(1.014558);

//         try
//     {
//         tf_box_to_robot.waitForTransform("/base_link", "/gazebo/link_states/box/box_link", ros::Time(0), ros::Duration(50.0)); // 查询tf树中base_link与box_link的关系
//     }
//     catch (tf::TransformException &ex)
//     {
//         ROS_ERROR("[adventure_tf]: (wait) %s", ex.what());
//         ros::Duration(1.0).sleep();
//     }

//     try
//     {
//         tf_box_to_robot.lookupTransform("/base_link", "/gazebo/link_states/box/box_link", ros::Time(0), (box_to_robot)); // 查询tf树中base_link与box_link的关系，查到后将其保存至this->camera_to_robot_
//     }
//     catch (tf::TransformException &ex)
//     {
//         ROS_ERROR("[adventure_tf]: (lookup) %s", ex.what());
//     }

//     box_robot_fram = box_to_robot * box_world_fram;

//     std::cout<<box_robot_fram <<std::endl;

//     return box_robot_fram;
    
// }

// 笛卡尔空间规划
int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");//设置显示中文，防止乱码
    // 初始化ros节点 创建线程类spinner(一个线程)用于处理回调函数
    ros::init(argc,argv,"moveit_Cartesian_demo");
    ros::AsyncSpinner spinner(1);
    // 启动线程 处理回调函数
    spinner.start();

    // 1.连接控制所需要的规划组
    // 包：：类：：对象：：方法
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    moveit::planning_interface::MoveGroupInterface gripper("gripper");

    cartesianPath cartesianPath_1;

    // 2.获取终端link的名称
    std::string end_effector_link = arm.getEndEffectorLink();
    // std::string gripper_name = gripper.getj;
    ROS_INFO("终端link的名称:%s",end_effector_link.c_str());
    // ROS_INFO("gripper的名称:%s", gripper_name.c_str());

    // 3.设置参考系
    std::string reference_fream = "base_link";
    arm.setPoseReferenceFrame(reference_fream);
    // gripper.setPoseReferenceFrame(reference_fream);

    // 4.允许运动规划后重新规划
    arm.allowReplanning(true);

    // 5.设置位置和姿态的允许误差
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.02);

    // 6.设置允许的最大速度和加速度
    arm.setMaxVelocityScalingFactor(0.2);
    arm.setMaxAccelerationScalingFactor(0.2);

    // 7.让机械臂回到up姿态
    arm.setNamedTarget("ready");
    arm.move();
    gripper.setNamedTarget("open");
    gripper.move();
    sleep(2);

    ROS_INFO("print now pose");
    printCurrentPose(arm.getCurrentPose().pose);

    geometry_msgs::Pose execute_point;
    execute_point = arm.getCurrentPose().pose;
    execute_point.position.z -= 1.03;//to base_link
    execute_point.position.z -= 0.1628;//sub the length of gripper

    std::vector<geometry_msgs::Pose> waypoints;       
    execute_point.position.z -= 0.385;//attach the box
    double pick_z = execute_point.position.z;

    //execute CartesianPath
    waypoints.push_back(execute_point);
    cartesianPath_1.doCartesianPath(waypoints);
    waypoints.clear();

    printCurrentPose(arm.getCurrentPose().pose);
    ros::WallDuration(3).sleep();

    // pick the box
    ROS_INFO("pick the box");
    gripper.setNamedTarget("close");
    gripper.move();
    ros::WallDuration(1).sleep();


    //up the box
    execute_point.position.z += 0.38;
    waypoints.push_back(execute_point);
    cartesianPath_1.doCartesianPath(waypoints);
    waypoints.clear();

    // right 
    execute_point.position.y += 0.5;
    waypoints.push_back(execute_point);
    cartesianPath_1.doCartesianPath(waypoints);
    waypoints.clear();

    // back
    execute_point.position.x  = -0.6;
    waypoints.push_back(execute_point);
    cartesianPath_1.doCartesianPath(waypoints);
    waypoints.clear();

    //down 
    execute_point.position.z = pick_z ;
    waypoints.push_back(execute_point);
    cartesianPath_1.doCartesianPath(waypoints);
    waypoints.clear();

    // place the box
    ROS_INFO("place the box");
    gripper.setNamedTarget("open");
    gripper.move();

    //up without box
    execute_point.position.z += 0.2;
    waypoints.push_back(execute_point);
    cartesianPath_1.doCartesianPath(waypoints);
    waypoints.clear();

    ros::WallDuration(2).sleep();

    // ready
    arm.setNamedTarget("ready");
    arm.move();

    ros::shutdown();
    return 0;
}
