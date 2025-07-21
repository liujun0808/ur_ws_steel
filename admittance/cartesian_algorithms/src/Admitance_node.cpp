#include <ros/ros.h>
#include "cartesian_algorithms/admittance/Admittance.h"

int main(int argc, char  **argv)
{
    ros::init(argc,argv,"Admitance_node");
    ros::NodeHandle nh;
    setlocale(LC_ALL,"");
    double frequency = 100;


    // 参数
    std::string topic_arm_state;
    std::string topic_arm_command;
    std::string topic_wrench_state;
    std::string base_link;
    std::string end_link;

    std::vector<double> M;
    std::vector<double> D;
    std::vector<double> K;
    std::vector<double> desired_pose;

    double arm_max_vel;
    double arm_max_acc;
    double twist_scale;

    // 从服务器中加载获取参数
    // topic names
    if (!nh.getParam("topic_arm_state",topic_arm_state)){ROS_ERROR("无法获取机械臂状态的话题名称");return -1;}
    if (!nh.getParam("topic_arm_command",topic_arm_command)){ROS_ERROR("无法获取机械臂命令的话题名称");return -1;}
    if (!nh.getParam("topic_wrench_state",topic_wrench_state)){ROS_ERROR("无法获取末端六维力的话题名称");return -1;}
    if (!nh.getParam("twist_scale",twist_scale)){ROS_ERROR("无法获取速度系数参数");return -1;}

    // 导纳参数获取
    if (!nh.getParam("mass_arm",M)){ROS_ERROR("无法获取导纳参数M");return -1;}
    if (!nh.getParam("damping_arm",D)){ROS_ERROR("无法获取导纳参数D");return -1;}
    if (!nh.getParam("stiffness_coupling",K)){ROS_ERROR("无法获取导纳参数K");return -1;}
    if (!nh.getParam("base_link",base_link)){ROS_ERROR("无法获取导纳参数base_link");return -1;}
    if (!nh.getParam("end_link",end_link)){ROS_ERROR("无法获取导纳参数end_link");return -1;}
    if (!nh.getParam("desired_pose",desired_pose)){ROS_ERROR("无法获取导纳参数desired_pose");return -1;}
    if (!nh.getParam("arm_max_vel",arm_max_vel)){ROS_ERROR("无法获取导纳参数arm_max_vel");return -1;}
    if (!nh.getParam("arm_max_acc",arm_max_acc)){ROS_ERROR("无法获取导纳参数arm_max_acc");return -1;}

    // 创建导纳控制器
    Admittance admittance(
        nh,frequency,topic_arm_state,
        topic_arm_command,topic_wrench_state,
        M,D,K,desired_pose,base_link,end_link,
        arm_max_vel,arm_max_acc
    );

    admittance.run();

    return 0;
}
