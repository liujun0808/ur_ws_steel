#include "ros/ros.h"
#include "moveit/move_group_interface/move_group_interface.h"//C++与move_group的接口
#include "moveit/robot_trajectory/robot_trajectory.h"

class cartesianPath
{
private:
    /* data */
    // ros::NodeHandle nh;

public:
    moveit::planning_interface::MoveGroupInterface arm;

    std::vector<geometry_msgs::Pose> waypoints;

    cartesianPath();

    void doCartesianPath(std::vector<geometry_msgs::Pose> waypoints);
};

