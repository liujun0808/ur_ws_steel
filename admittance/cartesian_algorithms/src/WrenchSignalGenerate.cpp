#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#define WRENCH_TOPIC    "/wrench_fake"
#define TOPIC_HZ    125.0

int main(int argc, char  **argv)
{
    ros::init(argc,argv,"WrenchSignalGenerate");
    ros::NodeHandle nh;

    ros::Publisher wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>(WRENCH_TOPIC,5);
    ros::Duration sleep(2.0);
    ros::Duration sleep2(5.0);
    ros::Rate loop_rate(TOPIC_HZ);
    sleep.sleep();

    geometry_msgs::WrenchStamped wrench_msg_init,wrench_msg;
    double t = 0.0;

    sleep.sleep();
    // while (nh.ok())
    // {
    //     if (static_cast<int>(t)%10 < 5)
    //     {
    //         wrench_msg.wrench.force.z = 1;
    //     }
    //     else
    //     {
    //         wrench_msg.wrench.force.z = -1;
    //     }
    //     t += 1/TOPIC_HZ;
    //     wrench_pub.publish(wrench_msg);
    //     loop_rate.sleep();
    // }

    wrench_msg.header.stamp = ros::Time::now();
    wrench_msg.wrench.force.z = 0;
    wrench_pub.publish(wrench_msg);

    sleep2.sleep();

    wrench_msg.header.stamp = ros::Time::now();
    wrench_msg.wrench.force.z = 1;
    wrench_pub.publish(wrench_msg);
    
    return 0;
}
