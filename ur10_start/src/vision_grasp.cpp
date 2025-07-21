#include "ur10_start/vision_grasp.h"

// 构造函数
vision_grasp::vision_grasp(ros::NodeHandle n_,float pregrasp_x,float pregrasp_y,float pregrasp_z,float length,float breadth)
:it_(n_),arm("manipulator"),gripper("gripper")//列表初始化vm(length,breadth) 
{
     this->nh_ = n_;
    // 监听相机与机器人的位置关系
    try
    {
        this->tf_camera_to_robot.waitForTransform("/base_link","/camera_link",ros::Time(0),ros::Duration(50));
    }
    catch(tf::TransformException & e)
    {
        std::cerr << e.what() << '\n';
    }

    //寻找关系并赋值
    try
    {
        this->tf_camera_to_robot.lookupTransform("/base_link","/camera_link",ros::Time(0),(this->camera_to_robot));
    }
    catch(tf::TransformException & e)
    {
        std::cerr << e.what() << '\n';
    }

    grasping = false;
    this->pregrasp_x = pregrasp_x;
    this->pregrasp_y = pregrasp_y;
    this->pregrasp_z = pregrasp_z;

    ros::AsyncSpinner spinner(1); 
    spinner.start();
    ready();
    ROS_INFO("等待5秒获取目标信息");
    ros::WallDuration(5).sleep();
    printCurrentPose(arm.getCurrentPose().pose);
    ros::WallDuration(1).sleep();
    // 订阅图像信息
    image_sub = it_.subscribe("/ur10/camera/image_raw",1,&vision_grasp::imageCb,this);
}
void vision_grasp::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    if (!grasping)
    {   
        try
        {
           cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception & e)
        {
            std::cerr << e.what() << '\n';
            return;
        }
        
    
    
        float obj_x,obj_y;
        // vm.get2DLocation(cv_ptr->image,obj_x,obj_y);
        // Temporary Debugging
        std::cout<< " X-Co-ordinate in Camera Frame :" << obj_x << std::endl;
        std::cout<< " Y-Co-ordinate in Camera Frame :" << obj_y << std::endl;

        // 将图像中点的位置转换为相机坐标系下的位置
        object_camera_frame.setX(0.835442);//相机距离目标顶面的垂直距离
        object_camera_frame.setY(-obj_x);
        object_camera_frame.setZ(-obj_y);

        obj_robot_frame = camera_to_robot * object_camera_frame;
        grasping = true;
        // Temporary Debugging
        std::cout<< " X-Co-ordinate in Robot Frame :" << obj_robot_frame.getX() << std::endl;
        std::cout<< " Y-Co-ordinate in Robot Frame :" << obj_robot_frame.getY() << std::endl;
        std::cout<< " Z-Co-ordinate in Robot Frame :" << obj_robot_frame.getZ() << std::endl;
    }
}

void vision_grasp::ready()
{   

    arm.setNamedTarget("ready");
    arm.move();
    
    gripper.setNamedTarget("open");
    gripper.move();

}

void vision_grasp::printCurrentPose(geometry_msgs::Pose currentPose)
{
    ROS_INFO("当前位置：%0.3f,%0.3f,%0.3f", currentPose.position.x,
             currentPose.position.y, currentPose.position.z-1.03);
    ROS_INFO("当前姿态四元数：%0.3f,%0.3f,%0.3f,%0.3f", currentPose.orientation.w,
             currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z);
}

// void vision_grasp::up()
// {
//     geometry_msgs::Pose up_point = arm.getCurrentPose().pose;


// }
void vision_grasp::pick()
{
    ros::WallDuration(1).sleep();
    gripper.setNamedTarget("close");
    gripper.move();

}
void vision_grasp::place()
{
    ros::WallDuration(1).sleep();
    gripper.setNamedTarget("open");
    gripper.move();

}

void vision_grasp::startGrasp()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("wait 2s....");   
    // gripper.setNamedTarget("open");
    // gripper.move();
    // ros::WallDuration(1).sleep();

    // ROS_INFO("1");
    // std::vector<double> current_joints =  arm.getCurrentJointValues();
    // ROS_INFO("2");
    // current_joints[0] += 1.57;

    // arm.setJointValueTarget(current_joints); 
    // ROS_INFO("3");
    // arm.move();
    // arm.setNamedTarget("ready");
    // arm.move();

    geometry_msgs::Pose current_pose = this->arm.getCurrentPose().pose;
    //定义抓取路点
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose pick_point;
    pick_point = current_pose;
    pick_point.position.z -=1.03;

    pick_point.position.y = obj_robot_frame.getY();
    waypoints.push_back(pick_point);


    pick_point.position.x = obj_robot_frame.getX();
    waypoints.push_back(pick_point);

    pick_point.position.z = (obj_robot_frame.getZ()+0.035);
    waypoints.push_back(pick_point);
    //执行笛卡尔轨迹
    ROS_INFO("执行抓取");
    cartesianPath1.doCartesianPath(waypoints);
    pick();
    double place_z = arm.getCurrentPose().pose.position.z - 1.03;
    ros::WallDuration(1).sleep();
    waypoints.clear();

    //定义放置路点
    geometry_msgs::Pose place_point;
    place_point = this->arm.getCurrentPose().pose;
    place_point.position.z -=1.03;//转为baselink下高度

    place_point.position.z  += 0.4;
    waypoints.push_back(place_point);

    place_point.position.x *= -1;
    waypoints.push_back(place_point);

    place_point.position.z = place_z;
    waypoints.push_back(place_point);


    //执行笛卡尔轨迹
    ROS_INFO("执行放置");
    cartesianPath1.doCartesianPath(waypoints);
    place();
    ros::WallDuration(1).sleep();
    waypoints.clear();

    //UP
    place_point.position.z +=0.2;
    waypoints.push_back(place_point);
    cartesianPath1.doCartesianPath(waypoints);
    waypoints.clear();

    ready();
    grasping = false;
}


int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"vision_grasp");
    ros::NodeHandle nh;

    float length, breadth, pregrasp_x, pregrasp_y, pregrasp_z;
    length = 1.5;
    breadth = 0.8;
    vision_grasp graspBox(nh,pregrasp_x,pregrasp_y,pregrasp_z,length,breadth);
    ROS_INFO("等待3s");
    ros::WallDuration(3).sleep();

    ros::spinOnce();
    graspBox.startGrasp();
    ros::WallDuration(60).sleep();
    
    ros::shutdown();
    return 0;
}
