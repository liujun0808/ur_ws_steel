#include <ros/ros.h>
#include "cartesian_algorithms/admittance/reinforcement.h"
#include "cartesian_algorithms/admittance/ibvs.h"

int main(int argc, char **argv)
{   
    ros::init(argc,argv,"test_search_hole");
    ros::NodeHandle nh;
    setlocale(LC_ALL,"");
    reinforcement reinforcement(nh);
    reinforcement.run();
    // ros::spin();

}