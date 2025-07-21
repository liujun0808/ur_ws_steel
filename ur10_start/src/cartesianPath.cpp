#include "ur10_start/cartesianPath.h"

cartesianPath::cartesianPath():arm("manipulator")
{

}

void cartesianPath::doCartesianPath(std::vector<geometry_msgs::Pose> waypoints)
{
    // std::string reference_fream = "base";
    // arm.setPoseReferenceFrame(reference_fream);
    // gripper.setPoseReferenceFrame(reference_fream);
    std::string reference_fream = "base";
    arm.setPoseReferenceFrame(reference_fream);
    // 4.允许运动规划后重新规划
    arm.allowReplanning(true);

    // 5.设置位置和姿态的允许误差
    arm.setGoalPositionTolerance(0.01);
    arm.setGoalOrientationTolerance(0.01);

    // 6.设置允许的最大速度和加速度
    arm.setMaxVelocityScalingFactor(0.1);
    arm.setMaxAccelerationScalingFactor(0.1);

    // 11.笛卡尔空间轨迹规划
        // 定义moveit类型中的robotTrajectory的msgs
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;//跳跃阈值,逆解不出来的点跳过
        const double eef_step  = 0.01;//终端步进值
        double fraction = 0.0;//路径规划覆盖率，覆盖路径列表中的所有点
        int maxtries = 100;//定义最大规划次数
        int attempts = 0;//定义已经尝试规划的次数
    while (fraction<1 && attempts <maxtries)
    {
        // 将路径列表中的位姿，和其他参数传入，把规划出来的轨迹放入trajectory
        fraction = arm.computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory);
        attempts++;
        if (attempts %10 ==0 )
        {
            ROS_INFO("规划%d次后依然在规划",attempts);
        }
    }
        if (fraction ==1)//表示规划成功
        {
            ROS_INFO("路径规划成功,执行规划");
            // 生成机械臂规划数据
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_=trajectory;
            //执行运动
            arm.execute(plan);
            sleep(2);
        }
        else{
            ROS_INFO("规划失败！规划了%d次,路径点覆盖率达到%0.4f",maxtries,fraction);
        }

}