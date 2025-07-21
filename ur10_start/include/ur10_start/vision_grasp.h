#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <tf/transform_listener.h>

#include "ur10_start/my_vision_manager.h"
#include "ur10_start/cartesianPath.h"

class vision_grasp
{
private:
    ros::NodeHandle nh_;

    geometry_msgs::Pose target_pose;

    moveit::planning_interface::MoveGroupInterface arm;

    moveit::planning_interface::MoveGroupInterface gripper;

    image_transport::ImageTransport it_;

    image_transport::Subscriber image_sub;

    bool grasping;

    cv_bridge::CvImagePtr cv_ptr;

    // my_vision_manager vm;
    
    // 变换矩阵
    tf::StampedTransform camera_to_robot;

    // 变换矩阵监听器
    tf::TransformListener tf_camera_to_robot;

    // 位置坐标
    tf::Vector3 object_camera_frame,obj_robot_frame;

    // 末端预抓取位置
    float pregrasp_x,pregrasp_y,pregrasp_z;

    cartesianPath cartesianPath1;

    //机械臂到达预抓取位置
    // void attainPosition(float pregrasp_x,float pregrasp_y,float pregrasp_z);

    void pick();

    // void up();
    // void down();
    // void left();
    // void right();
    // void forth();
    // void back();

    void place();

    void printCurrentPose(geometry_msgs::Pose currentPose);
public:
    // 构造函数
    vision_grasp(ros::NodeHandle n_,float pregrasp_x,float pregrasp_y,float pregrasp_z,float length=1.5,float breadth=0.8);

    // 回调函数 中获取目标位置,赋值给obj_robot_frame
    void imageCb(const sensor_msgs::ImageConstPtr &msg);

    void startGrasp();

    void ready();

};


