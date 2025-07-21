#include "cartesian_algorithms/admittance/reinforcement.h"
#include "geometry_msgs/PointStamped.h"

// 构造
reinforcement::reinforcement(ros::NodeHandle &n):nh_(n),arm("manipulator"),loop_rate(125),flag(0),ibvs_data(13) 
{
    yolo_sub_state = nh_.subscribe("/workpieces",1,&reinforcement::CB_get_workpieces,this);
    grasp_flag_pub =  nh_.advertise<yolov8_ros::grasp_flag>("grasping",1);
    ft_bias_client = nh_.serviceClient<netft_utils::SetBias>("/bias");  // ft置0
    client_sec_img =  nh_.serviceClient<yolov8_ros::center>("center"); // 二次拍照
    ibvs_srv =  nh_.serviceClient<yolov8_ros::ibvs>("ibvs_srv"); // ibvs请求服务
    // sub_wrench_state_ = nh_.subscribe("transformed_world_filter",5,
    // &reinforcement::state_wrench_callback,this);
    sub_wrench_state_ = nh_.subscribe("transformed_world",100,
    &reinforcement::state_wrench_callback,this);
    // admit_vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds", 1);// ibvs
    admit_vel_pub = nh_.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1);// ibvs-CBF
    tcp_pose = nh_.advertise<geometry_msgs::PoseStamped>("ft_dir",1);
    ser_vel = nh_.advertise<geometry_msgs::PointStamped>("ser_vel",1);
    hole_find_movestatus =nh_.subscribe("/execute_trajectory/feedback",5,&reinforcement::callback_status,this);
    ibvs_data_sub = nh_.subscribe("ibvs_data",5,&reinforcement::ibvs_callback,this);
    sub_joint_state = nh_.subscribe("/joint_states", 1, &reinforcement::jointStateCallback, this);
    // 力传感器 偏置服务信息
    setBias_client.request.toBias = true;
    setBias_client.request.forceMax = 20;
    setBias_client.request.torqueMax = 10;

    Py_Initialize();//Python解释器初始化
    gripper_connect();
    gripper_open();
    
    // 初始化
    cam_k<<607.5791016,0.0,313.58734131,  // D435i
            0.0,607.50048828,251.40821838,
            0.0,0.0,1;
    // cam_k<<606.1358642578125,0.0,317.83746337890625, // D435
    //     0.0,604.9152221679688,244.45687866210938,
    //     0.0,0.0,1;
    eye_to_hand<<0.99921847,0.02002815,-0.03407832,-0.01114759,
                -0.01984378,0.99978662,0.0057399,-0.10696188,
                0.03418601,-0.00505917,0.99940268,-0.13623781,
                0.0,0.0,0.0,1.0;
    workp_in_base_link.setZero();
    workp_in_camera.setZero();
    workp_in_pixel.setZero();
    workp_in_tool.setZero();
    tool_to_base_link.setZero();
    tool_in_base_link.setZero();
    // first_node.setZero();
    seconde_node.setZero();
    // seconde_node<<0.12347,0.8365,0.270;
    first_node<<0.12347,0.8365,0.0853;
    grasp_flag_msg.flag = false;
    // M 改变调节时间，且M太大会振荡，调节时间会加长；D改变超调，D越大超调越小但调节时间也会加长；K越大稳态误差越小（但在自适应变导纳中不使用K）
    M_<<10.0,0.0,0.0,
        0.0,10.0,0.0,
        0.0,0.0,10.0;
    D_<<280.0,0.0,0.0,
        0.0,280.0,0.0,
        0.0,0.0,280.0;
    K_<<0.0,0.0,0.0,
        0.0,0.0,0.0,
        0.0,0.0,0.0;
    now = std::time(nullptr); // 获取当前时间（秒）
    ibvs_start = false;    

}
reinforcement::~reinforcement(){Py_Finalize();} //关闭解释器


// ****************************************回调函数*********************************************
void reinforcement::CB_get_workpieces(const yolov8_ros::objectArrayConstPtr msg)
{   
    // std::cout<<"CB"<<std::endl;
    // grasp_flag_msg.flag = true;
    // grasp_flag_pub.publish(grasp_flag_msg);

    size_t count =  msg->objects.size();
    // 按类别分组
    for (size_t i = 0; i < count; i++)
    {
        if (msg->objects[i].cls == 0){ hor.objects.push_back(msg->objects[i]);} 
        if (msg->objects[i].cls == 1){ node.objects.push_back(msg->objects[i]);}
        if (msg->objects[i].cls == 2){ ver.objects.push_back(msg->objects[i]);}
        // 金属
        // if (msg->objects[i].cls == 2){ hor.objects.push_back(msg->objects[i]);} 
        // if (msg->objects[i].cls == 0){ node.objects.push_back(msg->objects[i]);}
        // if (msg->objects[i].cls == 1){ ver.objects.push_back(msg->objects[i]);}
    }


    std::sort(hor.objects.begin(),hor.objects.end(),compare_by_center_x ); // 将成员函数指针绑定到this对象上，传参使用_1站位符
    std::sort(node.objects.begin(),node.objects.end(),compare_by_center_x);
    std::sort(ver.objects.begin(),ver.objects.end(),compare_by_center_x);

    // for (size_t i = 0; i < ver.objects.size(); i++)
    // {
    //     std::cout << ver.objects[i].center_x << std::endl;
    // }
    

}
void reinforcement::state_wrench_callback(const geometry_msgs::WrenchStampedConstPtr msg)
{
    // wrench_ft_frame << msg->wrench.force.x,msg->wrench.force.y,msg->wrench.force.z; 
    wrench_ft_frame << 0.0,0.0,msg->wrench.force.z; 
    
    // wrench_ft_frame << 0,0,-1; 
}

void reinforcement::ibvs_callback(const std_msgs::Float64MultiArrayConstPtr & msg)
{   
    if (!ibvs_start)
    {   
        // ROS_INFO("ibvs no start!");
        return;
    }
    
    if (msg->data.size() != 12)
    {
        ROS_ERROR("Received invalid data size! Expected 12, got %zu", msg->data.size());
        ibvs_data[12] = -1.0;
        return;
    }
    for (size_t i = 0; i < msg->data.size(); i++)
    {
        ibvs_data[i] = msg->data[i];
        // std::cout<<ibvs_data[i]<<" "<<std::endl;
    }
    ibvs_data[12] = 1.0;
    // ROS_INFO("ibvs data ok!");
}

// ****************************************任务函数*********************************************
void reinforcement::run()
{   
    changeController("scaled_pos_joint_traj_controller","joint_group_vel_controller"); // 启动-停止
    ros::Duration sleep1(1);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // 位姿初始化
    arm.setNamedTarget("home");
    arm.move();

    gripper_open();
    int i = 0;

    while (ros::ok() && (hor.objects.size()!=4 || node.objects.size()!=4 || ver.objects.size()!=4))
    {
        std::cout<<"hor size:"<< hor.objects.size()<<std::endl;
        std::cout<<"node size:"<< node.objects.size()<<std::endl;
        std::cout<<"ver size:"<< ver.objects.size()<<std::endl;
        ros::Duration(1).sleep();
    }
    std::cout<<"输入1开始,其他退出"<<std::endl;
    std::cin>>i;
    if (i != 1)
    {
        return;
    }
    
    // 第一层node
    // first_layer_node();
    // std::cout<<"首个node的坐标"<<first_node<<std::endl;

    // 第一层horizontal
    // first_layer_hori();

    // 第一层vertical
    // first_layer_ver();

    // 第二层node
    // seconde_layer_node();

    // 第二层horizontal
    // seconde_layer_hori();


}

void reinforcement::first_layer_node()
{   
    gripper_open();
    std::size_t node_num = node.objects.size();
    vector3d node_in_pixel;
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose;
    yolov8_ros::center center;
    center.request.x = 320;
    center.request.y = 240;
    center.request.cls = 0;
    // std::size_t y=0;
    for (size_t i = 0; i < 2; i++)
    {   
        // 初始位置 姿态获取
        get_rotation_matrix(tool_to_base_link,tool_in_base_link,listener,"base","tool0");
        target_pose.position.x = tool_in_base_link[0];
        target_pose.position.y = tool_in_base_link[1];
        target_pose.position.z = tool_in_base_link[2];
        target_pose.orientation.x = tool_to_base_qua.getX();
        target_pose.orientation.y = tool_to_base_qua.getY();
        target_pose.orientation.z = tool_to_base_qua.getZ();
        target_pose.orientation.w = tool_to_base_qua.getW();

        if (node.objects[i].depth < 100){node.objects[i].depth = 653;std::cout<<"深度已经补偿"<<std::endl;} // 653 home下相机距离平面的高度
        std::cout<<node.objects[i].depth<<std::endl;
        node_in_pixel<<node.objects[i].center_x,node.objects[i].center_y,653;
        camera_to_base(node_in_pixel);

        // 移动至节点上方进行二次拍照
        target_pose.position.x = workp_in_base_link[0];
        waypoints.push_back(target_pose);
        target_pose.position.y = workp_in_base_link[1]-0.1069;
        waypoints.push_back(target_pose); 
        moveCartesian(waypoints);
        waypoints.clear();
        client_sec_img.call(center);
        ros::Duration sleep(1);
        // 判断二次拍照是否计算成功
        if (!center.response.flag){std::cout<<"二次拍照计算失败"<<std::endl;return;}
        if (center.response.depth < 100){center.response.depth = 653;std::cout<<"深度已经补偿"<<std::endl;} // 593 home下相机距离node的高度
        std::cout<<"response结果:"<<center.response<<std::endl;
        // 将像素坐标进行更新
        node_in_pixel<<center.response.x_r,center.response.y_r,653;
        camera_to_base(node_in_pixel);
        std::cout<<"二次拍照后的坐标:"<<workp_in_base_link<<std::endl;
        
        // 旋转抓取姿态
        // geometry_msgs::Quaternion orientation_temp = target_pose.orientation;
        // target_pose.orientation = rotation_grasp(center.response.angle,1);
        // 节点抓放
        target_pose.position.x = workp_in_base_link[0]-0.0152;
        waypoints.push_back(target_pose);
        target_pose.position.y = workp_in_base_link[1]-0.0232;
        waypoints.push_back(target_pose);
        target_pose.position.z = 0.06;
        waypoints.push_back(target_pose);
        moveCartesian(waypoints);
        waypoints.clear();
// 测试++++++++++++++++++++++++++
        // int y=0;
        // std::cout<<"输入1开始,其他退出"<<std::endl;
        // std::cin>>y;
        // if (y != 1)
        // {   
        //     return;
        // }
// 测试++++++++++++++++++++++++++++++
        gripper_close();
        sleep.sleep();
        // target_pose.orientation = orientation_temp; // 变化为原来的姿态
        target_pose.position.z += (0.15+0.095);
        waypoints.push_back(target_pose);
        // get_hole_pos(y);
        switch (i)
        {
        case 0:
            {
            target_pose.position.x -= 0.5;
            // target_pose.position.x -= 0.2;
            waypoints.push_back(target_pose);
            target_pose.position.y += 0.1;
            waypoints.push_back(target_pose);
            target_pose.position.z -= 0.147; // 0.147
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();

            // 自适应导纳控制
            // vector3d ft_dir(0.0,0.0,5.0);   
            // changeController("joint_group_vel_controller","scaled_pos_joint_traj_controller");
            // ft_bias_client.call(setBias_client);
            // sleep.sleep();
            // now = std::time(nullptr); // 获取当前时间（秒）
            // std::cout << "node放置开始时间: " << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S") << std::endl;
            // admittance(ft_dir,13.0);
            // now = std::time(nullptr); // 获取当前时间（秒）
            // std::cout << "node放置结束时间: " << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S") << std::endl;
            // changeController("scaled_pos_joint_traj_controller","joint_group_vel_controller"); // 启动-停止

            // 记录第一个node放置的位置
            get_rotation_matrix(tool_to_base_link,tool_in_base_link,listener,"base","tool0");
            first_node<<tool_in_base_link[0],tool_in_base_link[1],tool_in_base_link[2];
            }
            break;
        case 1:
            {target_pose.position.x = first_node[0];
            waypoints.push_back(target_pose);
            target_pose.position.y = first_node[1]-0.192;
            waypoints.push_back(target_pose);
            target_pose.position.z -= 0.147;
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            }
            break;
        case 2:
            {target_pose.position.x = first_node[0]+0.192;
            waypoints.push_back(target_pose);
            target_pose.position.y = first_node[1];
            waypoints.push_back(target_pose);
            target_pose.position.z -= 0.148;
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            }
            break;
        case 3:
            {target_pose.position.x = first_node[0]+0.192;
            waypoints.push_back(target_pose);
            target_pose.position.y = first_node[1]-0.192;
            waypoints.push_back(target_pose);
            target_pose.position.z -= 0.15;
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            }
            break;
        default:
            break;
        }
        sleep.sleep();
        gripper_open();
        sleep.sleep();

        target_pose.position.z += 0.1;
        waypoints.push_back(target_pose);
        moveCartesian(waypoints);
        waypoints.clear();
        arm.setNamedTarget("home");
        arm.move();
// ************************
        // ros::Duration(10).sleep();
        // ft_bias_client.call(setBias_client);
        // std::cout<<setBias_client.response.success<<std::endl;
// ************************  
    }    
}

void reinforcement::first_layer_hori()
{   
    // 测试
    ros::Publisher  ibvs_v_c_pub = nh_.advertise<geometry_msgs::TwistStamped>("/ibvs_v_c_pub", 1);
    ros::Publisher  ibvs_error_pub = nh_.advertise<geometry_msgs::PointStamped>("/ibvs_error", 1);
    geometry_msgs::PointStamped ibvs_error;
    // 测试
    gripper_open();
    std::size_t hor_num = hor.objects.size();
    vector3d hori_in_pixel;
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose;
    yolov8_ros::center center;
    yolov8_ros::ibvs ibvs;

    // std::size_t y=0;
    for (size_t i = 0; i < 1; i++)
    {   
        // 初始位置 姿态获取
        get_rotation_matrix(tool_to_base_link,tool_in_base_link,listener,"base","tool0");
        target_pose.position.x = tool_in_base_link[0];
        target_pose.position.y = tool_in_base_link[1];
        target_pose.position.z = tool_in_base_link[2];
        target_pose.orientation.x = tool_to_base_qua.getX();
        target_pose.orientation.y = tool_to_base_qua.getY();
        target_pose.orientation.z = tool_to_base_qua.getZ();
        target_pose.orientation.w = tool_to_base_qua.getW();

        if (hor.objects[i].depth < 100){hor.objects[i].depth = 653;std::cout<<"深度已经补偿"<<std::endl;} 
        std::cout<<hor.objects[i].depth<<std::endl;
        hori_in_pixel<<hor.objects[i].center_x,hor.objects[i].center_y,653;
        camera_to_base(hori_in_pixel);

        // 移动至节点上方进行二次拍照
        target_pose.position.x = workp_in_base_link[0]+0.01114759;
        waypoints.push_back(target_pose);
        target_pose.position.y = workp_in_base_link[1]-0.1069;
        waypoints.push_back(target_pose); 
        moveCartesian(waypoints);
        waypoints.clear();
        // 请求服务 若请求失败则返回的仍是原来的像素坐标
        center.request.x = hori_in_pixel[0];
        center.request.y = hori_in_pixel[1];
        center.request.cls = 2;
        client_sec_img.call(center);
        ros::Duration sleep(1);
        // 判断二次拍照是否计算成功
        while (!center.response.flag)
        {
            std::cout<<"二次拍照计算失败response.flag:"<<center.response.flag<<std::endl;
            client_sec_img.call(center);
            sleep.sleep();
        }
        if (center.response.depth < 100){center.response.depth = 653;std::cout<<"深度已经补偿"<<std::endl;} 
        std::cout<<"response结果:"<<center.response<<std::endl;
        // 将像素坐标进行更新
        hori_in_pixel<<center.response.x_r,center.response.y_r,653;
        camera_to_base(hori_in_pixel);
        std::cout<<"二次拍照后的坐标:"<<workp_in_base_link<<std::endl;
        // 测试++++++++++++++++++++++++++
            int m=0;
            std::cout<<"输入1开始,其他退出"<<std::endl;
            std::cin>>m;
            if (m != 1)
            {   
                return;
            }
        // 测试++++++++++++++++++++++++++++++
        // 旋转抓取姿态
        geometry_msgs::Quaternion orientation_temp = target_pose.orientation;
        target_pose.orientation = rotation_grasp(hor.objects[i].theta,0);
        // 横杆抓放
        target_pose.position.x = workp_in_base_link[0]-0.0152;
        waypoints.push_back(target_pose);
        target_pose.position.y = workp_in_base_link[1]-0.022;
        waypoints.push_back(target_pose);
        target_pose.position.z = 0.006;
        waypoints.push_back(target_pose);
        moveCartesian(waypoints);
        waypoints.clear();
        sleep.sleep();
        gripper_close();
        ros::Duration(2).sleep();
        target_pose.position.z += 0.20;
        waypoints.push_back(target_pose);
        moveCartesian(waypoints);
        waypoints.clear();
        target_pose.orientation = orientation_temp; // 变化为原来的姿态
        geometry_msgs::TwistStamped TwistStamped;
        std_msgs::Float64MultiArray q_dot_array;
        q_dot_array.data.resize(6);
        switch (i)
        {
        case 0:
            {target_pose.position.x = first_node[0] + 0.02;
            waypoints.push_back(target_pose);
            target_pose.position.y =first_node[1] - 0.098 - 0.03;
            waypoints.push_back(target_pose);
            // target_pose.position.z = first_node[2] + 0.006;
            // waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            // 测试++++++++++++++++++++++++++
            std::cout<<"输入2开始,其他退出"<<std::endl;
            std::cin>>m;
            if (m != 2)
            {   
                return;
            }
            // 测试++++++++++++++++++++++++++++++
            // 请求开启ibvs功能
            std::cout<<"Request to enable ibvs function!"<<std::endl;
            ibvs.request.ibvs_request = true;
            ibvs_srv.call(ibvs);
            sleep.sleep();
            ibvs_start = ibvs.response.ibvs_response;
            sleep.sleep();

            std::cout<<ibvs_start<<std::endl;
            changeController("joint_group_vel_controller","scaled_pos_joint_traj_controller"); // 启动-停止
            // for (size_t i = 0; i < 4; i++)
            // {
            //     std::cout<<"ibvs_data[12]:"<<ibvs_data[12]<<std::endl;
            //     vpColVector v_c_temp = ibvs_node.ibvs_run(ibvs_data);
            //     std::cout<<"ibvs_node.error:"<<ibvs_node.error<<std::endl;
            //     loop_rate.sleep();
            //     ros::spinOnce();
            // }
            double convergence_threshold = 0.1;
            bool swich= 1;
            while (!ibvs_node.has_converged && ros::ok())
            {   
                Eigen::VectorXd q_dot = ibvs_node.ibvs_run(ibvs_data,curr_joint_state);
                ibvs_error.point.x = ibvs_node.error;
                TwistStamped.header.stamp = ros::Time::now();
                ibvs_error.header.stamp = ros::Time::now();
                q_dot_array.data[0] = q_dot[0];
                q_dot_array.data[1] = q_dot[1];
                q_dot_array.data[2] = q_dot[2];
                q_dot_array.data[3] = q_dot[3];
                q_dot_array.data[4] = q_dot[4];
                q_dot_array.data[5] = q_dot[5];
                TwistStamped.twist.linear.x = q_dot[0];
                TwistStamped.twist.linear.y = q_dot[1];
                TwistStamped.twist.linear.z = q_dot[2];
                TwistStamped.twist.angular.x = q_dot[3];
                TwistStamped.twist.angular.y = q_dot[4];
                TwistStamped.twist.angular.z = q_dot[5];
                // TwistStamped.twist.linear.x = v_c[0]*(0.5);
                // TwistStamped.twist.linear.y = v_c[1]*(-0.5);
                // TwistStamped.twist.linear.z = v_c[2]*(-0.5);
                // TwistStamped.twist.angular.x = 0.0;
                // TwistStamped.twist.angular.y = 0.0;
                // TwistStamped.twist.angular.z = 0.0;
                admit_vel_pub.publish(q_dot_array);
                ibvs_v_c_pub.publish(TwistStamped);
                ibvs_error_pub.publish(ibvs_error);
                ros::spinOnce();
                // loop_rate.sleep();
                // 避免初始计算时产生的不正确误差
                if (ibvs_node.error > 100  &&  swich== 1)
                {
                    convergence_threshold = 0.8;
                    swich = 0; 
                }
                
                if (ibvs_node.error < convergence_threshold) 
                {
                    ibvs_node.has_converged = true;
                    // 速度回0
                    TwistStamped.header.stamp = ros::Time::now();
                    TwistStamped.twist.linear.x = 0.0;
                    TwistStamped.twist.linear.y = 0.0;
                    TwistStamped.twist.linear.z = 0.0;
                    TwistStamped.twist.angular.x = 0.0;
                    TwistStamped.twist.angular.y = 0.0;
                    TwistStamped.twist.angular.z = 0.0;
                    q_dot_array.data[0] = 0.0;
                    q_dot_array.data[1] = 0.0;
                    q_dot_array.data[2] = 0.0;
                    q_dot_array.data[3] = 0.0;
                    q_dot_array.data[4] = 0.0;
                    q_dot_array.data[5] = 0.0;
                    admit_vel_pub.publish(q_dot_array);
                    ibvs_v_c_pub.publish(TwistStamped);
                    std::cout<<"伺服完成"<<std::endl;
                    // 请求关闭ibvs功能
                    ibvs.request.ibvs_request = false;
                    ibvs_srv.call(ibvs);
                    ibvs_start = ibvs.response.ibvs_response;
                    sleep.sleep();                    
                }
            }
            changeController("scaled_pos_joint_traj_controller","joint_group_vel_controller"); // 启动-停止
            }
            break;
        case 1:
            {target_pose.position.x = first_node[0]+0.192;
            waypoints.push_back(target_pose);
            target_pose.position.y = first_node[1]-0.098+0.002;
            waypoints.push_back(target_pose);
            target_pose.position.z = first_node[2] + 0.005;
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            }
            break;
        case 2:
            {target_pose.position.x = first_node[0]+0.098+0.003;
            waypoints.push_back(target_pose);
            target_pose.position.y = first_node[1];
            waypoints.push_back(target_pose);
            target_pose.position.z = first_node[2]+0.008;
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            // 旋转
            movejoint6(1.570796327);
            }
            break;
        case 3:
            {target_pose.position.x = first_node[0]+0.098-0.001;
            waypoints.push_back(target_pose);
            target_pose.position.y = first_node[1]-0.192;
            waypoints.push_back(target_pose);
            target_pose.position.z = first_node[2]+0.008;
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            // 旋转
            movejoint6(1.570796327);
            }
            break;
        default:
            break;
        }
        // 测试++++++++++++++++++++++++++
            std::cout<<"输入3开始放置,其他退出"<<std::endl;
            std::cin>>m;
            if (m != 3)
            {   
                return;
            }
        // 更新末端位姿
        get_rotation_matrix(tool_to_base_link,tool_in_base_link,listener,"base","tool0");
        target_pose.position.x = tool_in_base_link[0];
        target_pose.position.y = tool_in_base_link[1];
        target_pose.position.z = tool_in_base_link[2];
        target_pose.orientation.x = tool_to_base_qua.getX();
        target_pose.orientation.y = tool_to_base_qua.getY();
        target_pose.orientation.z = tool_to_base_qua.getZ();
        target_pose.orientation.w = tool_to_base_qua.getW();
        // 往后-y方向移动2毫米
        target_pose.position.y -= 0.001;
        waypoints.push_back(target_pose);
        moveCartesian(waypoints);
        waypoints.clear();

         // 以速度控制放置横杆
        changeController("joint_group_vel_controller","scaled_pos_joint_traj_controller"); // 启动-停止
        ros::Time start_time = ros::Time::now();
        ros::Duration elapsed_time;
        while (ros::ok())
        {   
            elapsed_time = ros::Time::now() - start_time;
            if (elapsed_time.toSec() >= 3.2)
            {   
                TwistStamped.header.stamp = ros::Time::now();
                TwistStamped.twist.linear.z = 0.0;
                admit_vel_pub.publish(TwistStamped);
                break; // 跳出循环
            }
            TwistStamped.header.stamp = ros::Time::now();
            TwistStamped.twist.linear.z = -0.01;
            admit_vel_pub.publish(TwistStamped);
        }
        changeController("scaled_pos_joint_traj_controller","joint_group_vel_controller"); // 启动-停止

        // target_pose.position.z -= 0.03;
        // waypoints.push_back(target_pose);
        gripper_open();
        sleep.sleep();

        // 放置结束 回到初始位姿
        get_rotation_matrix(tool_to_base_link,tool_in_base_link,listener,"base","tool0");
        target_pose.position.x = tool_in_base_link[0];
        target_pose.position.y = tool_in_base_link[1];
        target_pose.position.z = tool_in_base_link[2];
        target_pose.orientation.x = tool_to_base_qua.getX();
        target_pose.orientation.y = tool_to_base_qua.getY();
        target_pose.orientation.z = tool_to_base_qua.getZ();
        target_pose.orientation.w = tool_to_base_qua.getW();
        target_pose.position.z += 0.1;
        waypoints.push_back(target_pose);
        moveCartesian(waypoints);
        waypoints.clear();
        arm.setNamedTarget("home");
        arm.move();
    }    
}

void reinforcement::first_layer_ver()
{
    gripper_open();
    ros::Duration sleep(1);
    std::size_t ver_num = ver.objects.size();
    vector3d ver_in_pixel;
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose;
    yolov8_ros::center center;
    center.request.x = 320;
    center.request.y = 240;
    center.request.cls = 1;
    double delta_x = 0.0;
    double delta_y = 0.0;
    // 初始位置 姿态获取
    for (size_t i = 0; i < 4; i++)
    {
        get_rotation_matrix(tool_to_base_link,tool_in_base_link,listener,"base","tool0");
        target_pose.position.x = tool_in_base_link[0];
        target_pose.position.y = tool_in_base_link[1];
        target_pose.position.z = tool_in_base_link[2];
        target_pose.orientation.x = tool_to_base_qua.getX();
        target_pose.orientation.y = tool_to_base_qua.getY();
        target_pose.orientation.z = tool_to_base_qua.getZ();
        target_pose.orientation.w = tool_to_base_qua.getW();
        
        if (ver.objects[i].depth < 640){ver.objects[i].depth = 641;std::cout<<"深度已经补偿"<<std::endl;} 
        ver_in_pixel<<ver.objects[i].center_x,ver.objects[i].center_y,641;
        std::cout<<"pixel:"<<ver_in_pixel<<std::endl;
        camera_to_base(ver_in_pixel);
        std::cout<<"base:"<<workp_in_base_link<<std::endl;

        // 移动至节点上方进行二次拍照
        target_pose.position.x = workp_in_base_link[0];
        waypoints.push_back(target_pose);
        target_pose.position.y = workp_in_base_link[1]-0.1069;
        waypoints.push_back(target_pose); 
        moveCartesian(waypoints);
        waypoints.clear();
        client_sec_img.call(center);
        sleep.sleep();
        // 判断二次拍照是否计算成功
        if (!center.response.flag){std::cout<<"二次拍照计算失败"<<std::endl;return;}
        if (center.response.depth < 100){center.response.depth = 653;std::cout<<"深度已经补偿"<<std::endl;} 
        std::cout<<"response结果:"<<center.response<<std::endl;

        // 将像素坐标进行更新
        ver_in_pixel<<center.response.x_r,center.response.y_r,653;
        camera_to_base(ver_in_pixel);
        std::cout<<"二次拍照后的坐标:"<<workp_in_base_link<<std::endl;
        
        // 旋转抓取姿态
        geometry_msgs::Quaternion orientation_temp = target_pose.orientation;
        target_pose.orientation = rotation_grasp(center.response.angle,2);
      
        // 抓取点非杆件中心值 需根据角度计算偏移值
        delta_x = cos(center.response.angle * 3.1415926 / 180.0) * 0.065;
        delta_y = sin(center.response.angle * 3.1415926 / 180.0) * 0.065;
        std::cout<<"delta_x:"<<delta_x<<std::endl;
        std::cout<<"delta_y:"<<delta_y<<std::endl;
        // 竖杆抓放
        if (center.response.angle < 0.0)
        {
            target_pose.position.x = workp_in_base_link[0]-0.0152 - delta_x;
            waypoints.push_back(target_pose);
            target_pose.position.y = workp_in_base_link[1]-0.0232 + delta_y;
            waypoints.push_back(target_pose);
            std::cout<<"center.response.angle < 0.0"<<std::endl;
        }else
        {
            target_pose.position.x = workp_in_base_link[0]-0.0152 - delta_x;
            waypoints.push_back(target_pose);
            target_pose.position.y = workp_in_base_link[1]-0.0232 + delta_y;
            waypoints.push_back(target_pose);
        }
        
        target_pose.position.z = 0.02;
        waypoints.push_back(target_pose);
        moveCartesian(waypoints);
        waypoints.clear();
// *************************测试*****************************
        // ros::Duration(3).sleep();
        // arm.setNamedTarget("home");
        // arm.move();
        // continue;
// *************************测试*****************************
        
        gripper_close();
        sleep.sleep();
        // 提升
        target_pose.position.z += 0.3;
        waypoints.push_back(target_pose);
        moveCartesian(waypoints);
        waypoints.clear();
        target_pose.orientation = orientation_temp;
        sleep.sleep();
        switch (i)
        {
        case 0:
            {target_pose.position.x = first_node[0];
            waypoints.push_back(target_pose);
            target_pose.position.y =first_node[1];
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            }break;
        case 1:
            {target_pose.position.x = first_node[0];
            waypoints.push_back(target_pose);
            target_pose.position.y =first_node[1]-0.192;
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            }break;
        case 2:
            {target_pose.position.x = first_node[0]+0.192;
            waypoints.push_back(target_pose);
            target_pose.position.y =first_node[1];
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            }break;
        case 3:
            {target_pose.position.x = first_node[0]+0.192;
            waypoints.push_back(target_pose);
            target_pose.position.y =first_node[1]-0.192;
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            }break;
        default:
            break;
        }
        rpy(1.570796327,0);
        get_rotation_matrix(tool_to_base_link,tool_in_base_link,listener,"base","tool0");
        target_pose.orientation.x = tool_to_base_qua.getX();
        target_pose.orientation.y = tool_to_base_qua.getY();
        target_pose.orientation.z = tool_to_base_qua.getZ();
        target_pose.orientation.w = tool_to_base_qua.getW();
        target_pose.position.z -= 0.08;
        waypoints.push_back(target_pose);
        moveCartesian(waypoints);
        waypoints.clear();
        sleep.sleep();

        // 接近孔件
        touch_hole();
        std::cout<<"是否需要寻找孔的标志："<<flag<<std::endl;

        now = std::time(nullptr); // 获取当前时间（秒）
        std::cout << "寻孔开始时间: " << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S") << std::endl;
        // 寻找孔
        if (flag == 1)
        {
            get_rotation_matrix(tool_to_base_link,tool_in_base_link,listener,"base","tool0");
            target_pose.position.x = tool_in_base_link[0];
            target_pose.position.y = tool_in_base_link[1];
            target_pose.position.z = tool_in_base_link[2];
            target_pose.orientation.x = tool_to_base_qua.getX();
            target_pose.orientation.y = tool_to_base_qua.getY();
            target_pose.orientation.z = tool_to_base_qua.getZ();
            target_pose.orientation.w = tool_to_base_qua.getW();
            space_arc_ver(target_pose);
        }
        // now = std::time(nullptr); // 获取当前时间（秒）
        // std::cout << "寻孔结束时间: " << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S") << std::endl;
        // 自适应导纳插孔
        vector3d ft_dir(0.0,0.0,5.0);   
        changeController("joint_group_vel_controller","scaled_pos_joint_traj_controller");
        ft_bias_client.call(setBias_client);
        sleep.sleep();
        admittance(ft_dir,8.0);
        // now = std::time(nullptr); // 获取当前时间（秒）
        // std::cout << "插孔结束时间: " << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S") << std::endl;
        changeController("scaled_pos_joint_traj_controller","joint_group_vel_controller"); // 启动-停止

        // 回home
        gripper_open();
        sleep.sleep();
        target_pose.position.z += 0.25;
        waypoints.push_back(target_pose);
        moveCartesian(waypoints);
        waypoints.clear();
        arm.setNamedTarget("home");
        arm.move();
    }
        
}

void reinforcement::seconde_layer_node()
{   
    gripper_open();
    vector3d node2_in_pixel;
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose;
    yolov8_ros::center center;
    center.request.x = 320;
    center.request.y = 240;
    center.request.cls = 0;
    // std::size_t y=0;
    for (size_t i = 4; i < 8; i++)
    {   
        // 初始位置 姿态获取
        get_rotation_matrix(tool_to_base_link,tool_in_base_link,listener,"base","tool0");
        target_pose.position.x = tool_in_base_link[0];
        target_pose.position.y = tool_in_base_link[1];
        target_pose.position.z = tool_in_base_link[2];
        target_pose.orientation.x = tool_to_base_qua.getX();
        target_pose.orientation.y = tool_to_base_qua.getY();
        target_pose.orientation.z = tool_to_base_qua.getZ();
        target_pose.orientation.w = tool_to_base_qua.getW();

        if (node.objects[i].depth < 100){node.objects[i].depth = 653;std::cout<<"深度已经补偿"<<std::endl;} // 653 home下相机距离平面的高度
        std::cout<<node.objects[i].depth<<std::endl;
        node2_in_pixel<<node.objects[i].center_x,node.objects[i].center_y,653;
        camera_to_base(node2_in_pixel);

        // 移动至节点上方进行二次拍照
        target_pose.position.x = workp_in_base_link[0];
        waypoints.push_back(target_pose);
        target_pose.position.y = workp_in_base_link[1]-0.1069;
        waypoints.push_back(target_pose); 
        moveCartesian(waypoints);
        waypoints.clear();
        client_sec_img.call(center);
        ros::Duration sleep(1);
        // 判断二次拍照是否计算成功
        if (!center.response.flag){std::cout<<"二次拍照计算失败"<<std::endl;return;}
        if (center.response.depth < 100){center.response.depth = 653;std::cout<<"深度已经补偿"<<std::endl;} // 593 home下相机距离node的高度
        std::cout<<"response结果:"<<center.response<<std::endl;
        // 将像素坐标进行更新
        node2_in_pixel<<center.response.x_r,center.response.y_r,653;
        camera_to_base(node2_in_pixel);
        std::cout<<"二次拍照后的坐标:"<<workp_in_base_link<<std::endl;
        
        // 旋转抓取姿态
        geometry_msgs::Quaternion orientation_temp = target_pose.orientation;
        target_pose.orientation = rotation_grasp(center.response.angle,1);
        // 节点抓放
        target_pose.position.x = workp_in_base_link[0]-0.0152;
        waypoints.push_back(target_pose);
        target_pose.position.y = workp_in_base_link[1]-0.0232;
        waypoints.push_back(target_pose);
        target_pose.position.z = 0.06;
        waypoints.push_back(target_pose);
        moveCartesian(waypoints);
        waypoints.clear();
// 测试++++++++++++++++++++++++++
        // std::cout<<"输入1开始,其他退出"<<std::endl;
        // std::cin>>y;
        // if (y != 1)
        // {
        //     return;
        // }
// 测试++++++++++++++++++++++++++++++
        gripper_close();
        sleep.sleep();
        target_pose.orientation = orientation_temp; // 变化为原来的姿态
        target_pose.position.z = 0.292;
        waypoints.push_back(target_pose);
        // get_hole_pos(y);
        switch (i)
        {
        case 4:
            {target_pose.position.x = first_node[0];
            waypoints.push_back(target_pose);
            target_pose.position.y = first_node[1];
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            }
            break;
        case 5:
            {target_pose.position.y = first_node[1]-0.192;
            waypoints.push_back(target_pose);
            target_pose.position.x = first_node[0];
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            }
            break;
        case 6:
            {
            target_pose.position.y = first_node[1];
            waypoints.push_back(target_pose);
            target_pose.position.x = first_node[0]+0.192;
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            }
            break;
        case 7:
            {target_pose.position.y = first_node[1]-0.192;
            waypoints.push_back(target_pose);
            target_pose.position.x = first_node[0]+0.192;
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            }
            break;
        default:
            break;
        }
        // 接触杆件
        touch_ver();
        // 搜点
        if (flag == 1)
        {
            get_rotation_matrix(tool_to_base_link,tool_in_base_link,listener,"base","tool0");
            target_pose.position.x = tool_in_base_link[0];
            target_pose.position.y = tool_in_base_link[1];
            target_pose.position.z = tool_in_base_link[2];
            target_pose.orientation.x = tool_to_base_qua.getX();
            target_pose.orientation.y = tool_to_base_qua.getY();
            target_pose.orientation.z = tool_to_base_qua.getZ();
            target_pose.orientation.w = tool_to_base_qua.getW();
            space_arc_node(target_pose);
        }
        // 自适应导纳插入
        std::cout<<"开始插孔......"<<"时间"<<ros::Time::now()<<std::endl;
        vector3d ft_dir(0.0,0.0,1.0);   
        changeController("joint_group_vel_controller","scaled_pos_joint_traj_controller");
        ft_bias_client.call(setBias_client);
        sleep.sleep();
        admittance(ft_dir,18);
        changeController("scaled_pos_joint_traj_controller","joint_group_vel_controller"); // 启动-停止
        std::cout<<"结束插孔......"<<"时间"<<ros::Time::now()<<std::endl;

        if (i==4)
        {
            get_rotation_matrix(tool_to_base_link,tool_in_base_link,listener,"base","tool0");
            seconde_node<<tool_in_base_link[0],tool_in_base_link[1],tool_in_base_link[2];
        }
        // 结束放置
        sleep.sleep();
        gripper_open();

        target_pose.position.z += 0.1;
        waypoints.push_back(target_pose);
        moveCartesian(waypoints);
        waypoints.clear();
        arm.setNamedTarget("home");
        arm.move();
// ************************
        // ros::Duration(10).sleep();
        // ft_bias_client.call(setBias_client);
        // std::cout<<setBias_client.response.success<<std::endl;
// ************************  
    }    
}

void reinforcement::seconde_layer_hori()
{
    gripper_open();
    vector3d hori_in_pixel;
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose;
    yolov8_ros::center center;
    center.request.x = 320;
    center.request.y = 240;
    center.request.cls = 2;
    // std::size_t y=0;
    for (size_t i = 4; i < 8; i++)
    {   
        // if (i==0 || i ==3)
        // {
        //    continue;
        // }
        
        // 初始位置 姿态获取
        get_rotation_matrix(tool_to_base_link,tool_in_base_link,listener,"base","tool0");
        target_pose.position.x = tool_in_base_link[0];
        target_pose.position.y = tool_in_base_link[1];
        target_pose.position.z = tool_in_base_link[2];
        target_pose.orientation.x = tool_to_base_qua.getX();
        target_pose.orientation.y = tool_to_base_qua.getY();
        target_pose.orientation.z = tool_to_base_qua.getZ();
        target_pose.orientation.w = tool_to_base_qua.getW();

        if (hor.objects[i].depth < 100){hor.objects[i].depth = 653;std::cout<<"深度已经补偿"<<std::endl;} 
        std::cout<<hor.objects[i].depth<<std::endl;
        hori_in_pixel<<hor.objects[i].center_x,hor.objects[i].center_y,653;
        camera_to_base(hori_in_pixel);

        // 移动至节点上方进行二次拍照
        target_pose.position.x = workp_in_base_link[0];
        waypoints.push_back(target_pose);
        target_pose.position.y = workp_in_base_link[1]-0.1069;
        waypoints.push_back(target_pose); 
        moveCartesian(waypoints);
        waypoints.clear();
        client_sec_img.call(center);
        ros::Duration sleep(1);
        // 判断二次拍照是否计算成功
        if (!center.response.flag){std::cout<<"二次拍照计算失败"<<std::endl;return;}
        if (center.response.depth < 100){center.response.depth = 653;std::cout<<"深度已经补偿"<<std::endl;} 
        std::cout<<"response结果:"<<center.response<<std::endl;
        // 将像素坐标进行更新
        hori_in_pixel<<center.response.x_r,center.response.y_r,653;
        camera_to_base(hori_in_pixel);
        std::cout<<"二次拍照后的坐标:"<<workp_in_base_link<<std::endl;
        
        // 旋转抓取姿态
        geometry_msgs::Quaternion orientation_temp = target_pose.orientation;
        target_pose.orientation = rotation_grasp(center.response.angle,0);
        // 横杆抓放
        target_pose.position.x = workp_in_base_link[0]-0.0152;
        waypoints.push_back(target_pose);
        target_pose.position.y = workp_in_base_link[1]-0.0232;
        waypoints.push_back(target_pose);
        target_pose.position.z = 0.023;
        waypoints.push_back(target_pose);
        moveCartesian(waypoints);
        waypoints.clear();

// 测试++++++++++++++++++++++++++
        // ros::Duration(3).sleep();
        // arm.setNamedTarget("home");
        // arm.move();
        // continue;
// 测试++++++++++++++++++++++++++++++
        gripper_close();
        sleep.sleep();
        target_pose.position.z = 0.362;
        waypoints.push_back(target_pose);
        moveCartesian(waypoints);
        waypoints.clear();
        target_pose.orientation = orientation_temp; // 变化为原来的姿态
        switch (i)
        {
        case 4:
            {target_pose.position.x = seconde_node[0];
            waypoints.push_back(target_pose);
            target_pose.position.y =seconde_node[1] - 0.098 +0.002;
            waypoints.push_back(target_pose);
            target_pose.position.z = seconde_node[2] + 0.008;
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            // // 记录第一个node放置的位置
            // get_rotation_matrix(tool_to_base_link,tool_in_base_link,listener,"base","tool0");
            // first_node<<tool_in_base_link[0],tool_in_base_link[1],tool_in_base_link[2];
            }
            break;
        case 5:
            {target_pose.position.x = seconde_node[0]+0.192;
            waypoints.push_back(target_pose);
            target_pose.position.y = seconde_node[1]-0.098+0.005;
            waypoints.push_back(target_pose);
            target_pose.position.z = seconde_node[2] + 0.008;
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            }
            break;
        case 6:
            {target_pose.position.x = seconde_node[0]+0.098;
            waypoints.push_back(target_pose);
            target_pose.position.y = seconde_node[1]+0.002;
            waypoints.push_back(target_pose);
            target_pose.position.z = seconde_node[2]+0.01;
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            // 旋转
            movejoint6(1.570796327);
            }
            break;
        case 7:
            {target_pose.position.x = seconde_node[0]+0.098;
            waypoints.push_back(target_pose);
            target_pose.position.y = seconde_node[1]-0.192;
            waypoints.push_back(target_pose);
            target_pose.position.z = seconde_node[2]+0.01;
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            // 旋转
            movejoint6(1.570796327);
            }
            break;
        default:
            break;
        }
        gripper_open();
        sleep.sleep();
     

        target_pose.position.z += 0.1;
        waypoints.push_back(target_pose);
        moveCartesian(waypoints);
        waypoints.clear();
        arm.setNamedTarget("home");
        arm.move();
// ************************
        // ros::Duration(10).sleep();
        // ft_bias_client.call(setBias_client);
        // std::cout<<setBias_client.response.success<<std::endl;
// ************************  
    }    
}
// ****************************************功能函数*********************************************
// 获得工件的X像素坐标，用以排序
bool reinforcement::compare_by_center_x(const yolov8_ros::workpieces& obj1,const yolov8_ros::workpieces& obj2)
{   
    
    return obj1.center_x < obj2.center_x;
}
void reinforcement::gripper_connect()
{
    PyRun_SimpleString("from jodellSdk.jodellSdkDemo import ClawEpgTool");
    PyRun_SimpleString("clawTool = ClawEpgTool()");
    PyRun_SimpleString("comlist = clawTool.searchCom()");
    PyRun_SimpleString("flag_connect = clawTool.serialOperation('/dev/ttyUSB0', 115200, True)");
    PyRun_SimpleString("\nflag_enable = clawTool.clawEnable(9, True)");
}
void reinforcement::gripper_close()
{
    PyRun_SimpleString("clawTool.runWithoutParam(9, 2)");
}
void reinforcement::gripper_open()
{
    PyRun_SimpleString("clawTool.runWithoutParam(9, 1)");
}
void reinforcement::moveCartesian(std::vector<geometry_msgs::Pose> waypoints){

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
            // sleep(2);
        }
        else{
            ROS_INFO("规划失败！规划了%d次,路径点覆盖率达到%0.4f",maxtries,fraction);
        }
}
void reinforcement::changeController(std::string start, std::string stop){
  ros::AsyncSpinner spinner(1);  
  controller_manager_msgs::SwitchController Controller;
  Controller.request.start_controllers.push_back(start);
  Controller.request.stop_controllers.push_back(stop);
  Controller.request.strictness = 2;
  ros::service::call("/controller_manager/switch_controller",Controller);  
}
void reinforcement::admittance(vector3d &ft_dir,const double &threshold_z)
{   

    Matrix3d S; // 选择矩阵
    S<<0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0;
    vector3d phi;
    vector3d arm_desired_twist_adm_;
    vector3d arm_desired_accelaration;
    vector3d last_wrench_ft,last_ft_dir;
    // float sigma = 0.0005f;
    float sigma = 0.001f;
    // float sigma = 0.005f;

    geometry_msgs::TwistStamped TwistStamped;
    float flag_arm_desired_twist_adm_ = 0.0f;
    arm_desired_twist_adm_.setZero();
    arm_desired_accelaration.setZero();
    phi.setZero();
    last_wrench_ft.setZero();
    last_ft_dir.setZero();
    geometry_msgs::PoseStamped tcp_pose_msg;
    geometry_msgs::PointStamped ser_vel_msg;
    ros::Duration duration = loop_rate.expectedCycleTime(); //得到循环一次的时间 （eg：1s执行125次，即125Hz，得到每次的执行时间0.008s）
    // while (ros::ok() && abs(wrench_ft_frame[2])<threshold_z)
    while (ros::ok())
    {   
        // 自适应导纳控制-变阻尼
        arm_desired_accelaration = M_.inverse() *(wrench_ft_frame - ft_dir - D_ * arm_desired_twist_adm_ - D_ * phi -sigma*(last_ft_dir - last_wrench_ft)); //  M*末端加速度 + D*末端速度 + K*末端位姿 = 外部力 - 期望力（0）
        arm_desired_twist_adm_ += arm_desired_accelaration * duration.toSec(); // 离散域的积分 累加：dv/dt = a ----》dv = a*dt  在进行累加，得到期望的末端速度
        phi = phi + sigma * (D_.inverse())* ( ft_dir - wrench_ft_frame) ;
        last_wrench_ft = wrench_ft_frame;
        last_ft_dir = ft_dir;

        TwistStamped.header.stamp = ros::Time::now();
        tcp_pose_msg.header.stamp = ros::Time::now();
        // ser_vel_msg.header.stamp = ros::Time::now();
        tcp_pose_msg.pose.position.z = ft_dir[2];
        TwistStamped.twist.linear.x = 0.0;
        TwistStamped.twist.linear.y = 0.0;
        // TwistStamped.twist.linear.x = arm_desired_twist_adm_[0];
        // TwistStamped.twist.linear.y = arm_desired_twist_adm_[1];
        TwistStamped.twist.linear.z = arm_desired_twist_adm_[2];
        TwistStamped.twist.angular.x = 0.0;
        TwistStamped.twist.angular.y = 0.0;
        TwistStamped.twist.angular.z = 0.0;
        ser_vel_msg.point.z = arm_desired_twist_adm_[2];
        // std::cout<<"ft_dir[2]:"<<ft_dir[2]<<std::endl;
        // ser_vel.publish(ser_vel_msg);
        admit_vel_pub.publish(TwistStamped);
        tcp_pose.publish(tcp_pose_msg);

        if (abs(wrench_ft_frame[2]- 12)<0.05)
        // if (abs(wrench_ft_frame[2] - ft_dir[2]) < 0.02)
        {   
            TwistStamped.header.stamp = ros::Time::now();
            TwistStamped.twist.linear.x = 0.0;
            TwistStamped.twist.linear.y = 0.0;
            TwistStamped.twist.linear.z = 0.0;
            TwistStamped.twist.angular.x = 0.0;
            TwistStamped.twist.angular.y = 0.0;
            TwistStamped.twist.angular.z = 0.0;
            admit_vel_pub.publish(TwistStamped);
            break;
        }
        ft_dir[2] += 0.004; // 每秒增加0.5N
        ros::spinOnce();
        loop_rate.sleep();
           
    }
    // 跳出循环后 初始化该话题信息
    TwistStamped.header.stamp = ros::Time::now();
    TwistStamped.twist.linear.x = 0.0;
    TwistStamped.twist.linear.y = 0.0;
    TwistStamped.twist.linear.z = 0.0;
    TwistStamped.twist.angular.x = 0.0;
    TwistStamped.twist.angular.y = 0.0;
    TwistStamped.twist.angular.z = 0.0;
    admit_vel_pub.publish(TwistStamped);


}
void reinforcement::rpy(double r,int i)
{   
    double rx=0;
    double ry=0;
    double rz=0;

    switch (i) {
        case 0:
            rx = r;
            break;
        case 1:
            ry = r;
            break;
        case 2:
            rz = r;
            break;
        default:
            break;
    }

    arm.setPoseReferenceFrame("base_link");
    std::vector<double> currentRPY = arm.getCurrentRPY();
    geometry_msgs::Pose target_pose;
    target_pose = arm.getCurrentPose().pose;

    Eigen::Vector3d eulerAngle(currentRPY[0]+rx,currentRPY[1]+ry,currentRPY[2]+rz);
    Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(0),Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1),Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(2),Vector3d::UnitZ()));
    Eigen::Quaterniond quaternion;
    quaternion=yawAngle*pitchAngle*rollAngle;
    target_pose.orientation.w = quaternion.w();
    target_pose.orientation.x = quaternion.x();
    target_pose.orientation.y = quaternion.y();
    target_pose.orientation.z = quaternion.z();
    arm.setPoseTarget(target_pose);
    arm.move();
    
}
void reinforcement::touch_hole()
{   
    flag = 0;
    ft_bias_client.call(setBias_client);
    geometry_msgs::TwistStamped TwistStamped;
    geometry_msgs::PointStamped ser_vel_msg;
    changeController("joint_group_vel_controller","scaled_pos_joint_traj_controller"); // 启动-停止
    ros::Duration sleep(1);
    ros::Duration elapsed_time;
    double wrench_ft_frame_flag = 0.0;
    ros::Time start_time = ros::Time::now();
    while (ros::ok())
    {   
        // std::cout<<wrench_ft_frame[2]<<std::endl;
        elapsed_time = ros::Time::now() - start_time;

        if (elapsed_time.toSec() >= 2.5)
        {
            break; // 跳出循环
        }
        TwistStamped.header.stamp = ros::Time::now();
        TwistStamped.twist.linear.x = 0.0;
        TwistStamped.twist.linear.y = 0.0;
        TwistStamped.twist.linear.z = -0.04;
        TwistStamped.twist.angular.x = 0.0;
        TwistStamped.twist.angular.y = 0.0;
        TwistStamped.twist.angular.z = 0.0;
        admit_vel_pub.publish(TwistStamped);
        ros::spinOnce();
        loop_rate.sleep(); 
    }
    std::cout<<wrench_ft_frame[2]<<std::endl;
    if ((wrench_ft_frame[2])>5)
    {
        // 需要搜孔
        flag = 1;
    }

    TwistStamped.header.stamp = ros::Time::now();
    TwistStamped.twist.linear.z = 0.0;
    admit_vel_pub.publish(TwistStamped);
    sleep.sleep();

    changeController("scaled_pos_joint_traj_controller","joint_group_vel_controller"); // 启动-停止
    ROS_INFO("touch over");
}
void reinforcement::fixtrajectorytime(moveit_msgs::RobotTrajectory &trajectory){
  moveit_msgs::RobotTrajectory trajectory2;
  std::size_t  WayPointCount = trajectory.joint_trajectory.points.size();
  trajectory2.joint_trajectory.points.push_back(trajectory.joint_trajectory.points[0]);
  for(std::size_t i=1; i < WayPointCount; i++){
    if (trajectory.joint_trajectory.points[i].time_from_start.sec > trajectory.joint_trajectory.points[i-1].time_from_start.sec){
      trajectory2.joint_trajectory.points.push_back(trajectory.joint_trajectory.points[i]);
    }
    else{
      if (trajectory.joint_trajectory.points[i].time_from_start.nsec > trajectory.joint_trajectory.points[i-1].time_from_start.nsec){
	trajectory2.joint_trajectory.points.push_back(trajectory.joint_trajectory.points[i]);  
      }
      else{
	ROS_INFO("Removed trajectory.point[%d]=%d.%ds ([%d]=%d.%ds) ",i,trajectory.joint_trajectory.points[i].time_from_start.sec,trajectory.joint_trajectory.points[i].time_from_start.nsec,i-1,trajectory.joint_trajectory.points[i-1].time_from_start.sec,trajectory.joint_trajectory.points[i-1].time_from_start.nsec);
      }
    }
  }
  trajectory.joint_trajectory.points.clear();
  trajectory.joint_trajectory.points = trajectory2.joint_trajectory.points;
}
geometry_msgs::Quaternion reinforcement::rotation_grasp(double rz,int cls)
{   
    double rz_rad = 0.0;
    if (cls == 1)
    {
        rz_rad= rz *(3.1415926/180.0) * (-1.0);
    }else
    {
        rz_rad= (90-rz) * (3.1415926/180.0);
    }
    
    arm.setPoseReferenceFrame("base_link");
    std::vector<double> currentRPY = arm.getCurrentRPY();
    // geometry_msgs::Pose target_pose;
    // target_pose = arm.getCurrentPose().pose;
    geometry_msgs::Quaternion orientation;

    Eigen::Vector3d eulerAngle(currentRPY[0],currentRPY[1],rz_rad);
    Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(0),Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1),Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(2),Vector3d::UnitZ()));
    Eigen::Quaterniond quaternion;
    quaternion=yawAngle*pitchAngle*rollAngle;
    orientation.w = quaternion.w();
    orientation.x = quaternion.x();
    orientation.y = quaternion.y();
    orientation.z = quaternion.z();
    return orientation;
}
bool reinforcement::get_rotation_matrix(Matrix3d & rotation_matrix,vector3d & position_,tf::TransformListener & listener,std::string from_frame,  std::string to_frame)
{
    tf::StampedTransform transform;
    tf::Vector3 position;
    try
    {
        listener.lookupTransform(from_frame,to_frame,ros::Time(0),transform);
        tool_to_base_qua = transform.getRotation();
        tf::matrixTFToEigen(transform.getBasis(),rotation_matrix);  // 获取旋转的部分getBasis
        position = transform.getOrigin(); // 获取位置部分
        position_ << position.getX(),position.getY(),position.getZ();
        // 测试
        // std::cout<<"tool0到base——link"<<rotation_matrix<<std::endl;
        // std::cout<<"tool0在base——link下"<<position_<<std::endl;
    }
    catch(tf::TransformException ex)
    {
        rotation_matrix.setZero();
        position_.setZero();
        ROS_WARN_STREAM_THROTTLE(1,"Wating for tf from: "<< from_frame << "to" << to_frame);// 具有节流功能的警告信息输出，1为两次输出的时间间隔
        return false; 
    }
    return true;

}
void reinforcement::camera_to_base(vector3d p_)
{   
    p_(2) *= 0.001;
    workp_in_camera(0) = (p_(0) - cam_k(0,2)) * (p_(2) / cam_k(0,0));
    workp_in_camera(1) = (p_(1) - cam_k(1,2)) * (p_(2) / cam_k(1,1));
    workp_in_camera(2) = p_(2);
    workp_in_tool = (eye_to_hand.block<3,3>(0,0)) * workp_in_camera + (eye_to_hand.block<3,1>(0,3));
    
    get_rotation_matrix(tool_to_base_link,tool_in_base_link,listener,"base","tool0");
    workp_in_base_link = tool_to_base_link * workp_in_tool + tool_in_base_link;
    // 测试
    // std::cout<<"工件像素坐标"<<p_<<std::endl;
    // std::cout<<"工件在相机坐标系下"<<workp_in_camera<<std::endl;
    // std::cout<<"工件在工具坐标系下"<<workp_in_tool<<std::endl;
    // std::cout<<"工件在base坐标系下"<<workp_in_base_link<<std::endl;
}
void reinforcement::movejoint6(double rad)
{
    std::vector<double> joint_list =  arm.getCurrentJointValues();
    joint_list[5] += rad;
    arm.setJointValueTarget(joint_list);
    arm.move();
    ros::Duration(1).sleep();
}
bool reinforcement::space_arc_ver(geometry_msgs::Pose &target_pose)
{   
    double radians = 30 * M_PI / 180.0; // 将角度转换为弧度
    double r = target_pose.position.z - first_node[2];
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose_arc;
    Eigen::Quaterniond quaternion;
    target_pose_arc.orientation = target_pose.orientation;
    Vector4d fz;
    geometry_msgs::TwistStamped TwistStamped;
    while (ros::ok() && (wrench_ft_frame[2])>4.0)
    {   
        get_rotation_matrix(tool_to_base_link,tool_in_base_link,listener,"base","tool0");
        target_pose.position.x = tool_in_base_link[0];
        target_pose.position.y = tool_in_base_link[1];
        target_pose.position.z = tool_in_base_link[2];
        ros::Duration sleep(1);
        arm.setPoseReferenceFrame("base");
        fz.setZero();
        for (int j = 0; j < 4; j++)
        {   
            quaternion.w() = target_pose.orientation.w;
            quaternion.x() = target_pose.orientation.x;
            quaternion.y() = target_pose.orientation.y;
            quaternion.z() = target_pose.orientation.z;
            // ft_bias_client.call(setBias_client);
            // sleep.sleep();
            switch (j)
            {
                case 0: // x+
                    {Eigen::Quaterniond rotation_y_add(Eigen::AngleAxisd(0.017453293, Eigen::Vector3d::UnitY()));
                    target_pose_arc.position.y = tool_in_base_link[1];
                    for (size_t i = 1; i < 8; i++)
                    {   
                        radians = i * M_PI / 180.0;
                        target_pose_arc.position.x = tool_in_base_link[0] +sin(radians)*r; 
                        target_pose_arc.position.z = tool_in_base_link[2] - (1-cos(radians))*r; 
                        quaternion = rotation_y_add * quaternion;
                        target_pose_arc.orientation.w = quaternion.w();
                        target_pose_arc.orientation.x = quaternion.x();
                        target_pose_arc.orientation.y = quaternion.y();
                        target_pose_arc.orientation.z = quaternion.z();
                        // target_pose_arc.position.x = p_x ;
                        // target_pose_arc.position.z = p_z ;
                        waypoints.push_back(target_pose_arc); 
                        // std::cout<<target_pose_arc<<std::endl;
                    }}
                    break;
                case 1: // x-
                    {target_pose_arc.position.y = tool_in_base_link[1];
                    Eigen::Quaterniond rotation_y_sub(Eigen::AngleAxisd(-0.017453293, Eigen::Vector3d::UnitY()));
                    for (size_t i = 1; i < 8; i++)
                    {   
                        radians = i * M_PI / 180.0;
                        target_pose_arc.position.x = tool_in_base_link[0] -sin(radians)*r; 
                        target_pose_arc.position.z = tool_in_base_link[2] - (1-cos(radians))*r; 
                        quaternion = rotation_y_sub * quaternion;
                        target_pose_arc.orientation.w = quaternion.w();
                        target_pose_arc.orientation.x = quaternion.x();
                        target_pose_arc.orientation.y = quaternion.y();
                        target_pose_arc.orientation.z = quaternion.z();
                        // target_pose_arc.position.x = p_x ;
                        // target_pose_arc.position.z = p_z ;
                        waypoints.push_back(target_pose_arc); 
                        // std::cout<<target_pose_arc<<std::endl;
                    }}
                    break;
                case 2: // y+
                    {target_pose_arc.position.x = tool_in_base_link[0];
                    Eigen::Quaterniond rotation_x_sub(Eigen::AngleAxisd(-0.017453293, Eigen::Vector3d::UnitX()));
                    for (size_t i = 1; i < 8; i++)
                    {   
                        radians = i * M_PI / 180.0;
                        target_pose_arc.position.y = tool_in_base_link[1] +sin(radians)*r; 
                        target_pose_arc.position.z = tool_in_base_link[2] - (1-cos(radians))*r; 
                        quaternion = rotation_x_sub * quaternion;
                        target_pose_arc.orientation.w = quaternion.w();
                        target_pose_arc.orientation.x = quaternion.x();
                        target_pose_arc.orientation.y = quaternion.y();
                        target_pose_arc.orientation.z = quaternion.z();
                        // target_pose_arc.position.y = p_y ;
                        // target_pose_arc.position.z = p_z ;
                        waypoints.push_back(target_pose_arc); 
                        // std::cout<<target_pose_arc<<std::endl;
                    }}
                    break;
                case 3: // y-
                    {target_pose_arc.position.x = tool_in_base_link[0];
                    Eigen::Quaterniond rotation_x_add(Eigen::AngleAxisd(0.017453293, Eigen::Vector3d::UnitX()));
                    for (size_t i = 1; i < 8; i++)
                    {   
                        radians = i * M_PI / 180.0;
                        target_pose_arc.position.y = tool_in_base_link[1] -sin(radians)*r; 
                        target_pose_arc.position.z = tool_in_base_link[2] - (1-cos(radians))*r; 
                        quaternion = rotation_x_add * quaternion;
                        target_pose_arc.orientation.w = quaternion.w();
                        target_pose_arc.orientation.x = quaternion.x();
                        target_pose_arc.orientation.y = quaternion.y();
                        target_pose_arc.orientation.z = quaternion.z();
                        // target_pose_arc.position.y = p_y ;
                        // target_pose_arc.position.z = p_z ;
                        waypoints.push_back(target_pose_arc); 
                        // std::cout<<target_pose_arc<<std::endl;
                    }}
                    break;
                default:
                    break;
            }
            std::cout<<"路点计算完成"<<std::endl;
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.2*0.005;
            arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            robot_trajectory::RobotTrajectory rt(arm.getCurrentState()->getRobotModel(), "manipulator");
            rt.setRobotTrajectoryMsg(*arm.getCurrentState(), trajectory);
            
            trajectory_processing::IterativeParabolicTimeParameterization iptp;
            iptp.computeTimeStamps(rt, 0.05, 1.0);
            rt.getRobotTrajectoryMsg(trajectory);  
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            fixtrajectorytime(trajectory);
            my_plan.trajectory_ = trajectory;
            arm.asyncExecute(my_plan);
            while(mvgr_status.status.status == 3 && ros::ok()){}  // since the topic is to slow, we need to wait till it switches from 3(executed) to 1(in motion)
            while(mvgr_status.status.status != 3 && ros::ok()){fz[j] = wrench_ft_frame[2];} 
            waypoints.clear();
            sleep.sleep();
            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
            ft_bias_client.call(setBias_client);
            sleep.sleep();
        }
        std::cout<<fz<<std::endl;
        int minIndex = 4;
        fz.minCoeff(&minIndex);
        std::cout<<minIndex<<std::endl;
        switch (minIndex)
        {
            case 0:
                {   
                    target_pose.position.z += 0.01;
                    waypoints.push_back(target_pose);
                    target_pose.position.x += 0.002;
                    waypoints.push_back(target_pose);
                    moveCartesian(waypoints);
                    waypoints.clear();
                }
                break;
            case 1:
                {
                    target_pose.position.z += 0.01;
                    waypoints.push_back(target_pose);
                    target_pose.position.x -= 0.002;
                    waypoints.push_back(target_pose);
                    moveCartesian(waypoints);
                    waypoints.clear();
                }
                break;
            case 2:
                {   target_pose.position.z += 0.01;
                    waypoints.push_back(target_pose);
                    target_pose.position.y += 0.002;
                    waypoints.push_back(target_pose);
                    moveCartesian(waypoints);
                    waypoints.clear();
                }
                break;
            case 3:
                {   target_pose.position.z += 0.01;
                    waypoints.push_back(target_pose);
                    target_pose.position.y -= 0.002;
                    waypoints.push_back(target_pose);
                    moveCartesian(waypoints);
                    waypoints.clear();
                }
                break;    
            default:
                break;
        }
        sleep.sleep();
        changeController("joint_group_vel_controller","scaled_pos_joint_traj_controller"); // 启动-停止
        ros::Duration elapsed_time;
        double wrench_ft_frame_flag = 0.0;
        ros::Time start_time = ros::Time::now();
        while (ros::ok())
        {   
            // std::cout<<wrench_ft_frame[2]<<std::endl;
            elapsed_time = ros::Time::now() - start_time;

            if (elapsed_time.toSec() >= 3)
            {
                break; // 跳出循环
            }
            TwistStamped.header.stamp = ros::Time::now();
            TwistStamped.twist.linear.x = 0.0;
            TwistStamped.twist.linear.y = 0.0;
            TwistStamped.twist.linear.z = -0.04;
            TwistStamped.twist.angular.x = 0.0;
            TwistStamped.twist.angular.y = 0.0;
            TwistStamped.twist.angular.z = 0.0;
            admit_vel_pub.publish(TwistStamped);
            ros::spinOnce();
            loop_rate.sleep(); 
        }
        TwistStamped.header.stamp = ros::Time::now();
        TwistStamped.twist.linear.z = 0.0;
        admit_vel_pub.publish(TwistStamped);
        sleep.sleep();
        changeController("scaled_pos_joint_traj_controller","joint_group_vel_controller"); // 启动-停止
        std::cout<<"Z轴受力:"<<wrench_ft_frame[2]<<std::endl;
    }
    
    // 判断是否到孔中心
    if ((wrench_ft_frame[2])<4.0)
    {   
        std::cout<<"已在孔中心"<<std::endl;
        return true;
    }else
    {
        std::cout<<"未在孔中心"<<std::endl;
        return false;
    }
    
}
void reinforcement::touch_ver()
{     
    flag = 0;
    ft_bias_client.call(setBias_client);
    geometry_msgs::TwistStamped TwistStamped;
    geometry_msgs::PointStamped ser_vel_msg;
    changeController("joint_group_vel_controller","scaled_pos_joint_traj_controller"); // 启动-停止
    ros::Duration sleep(1);
    sleep.sleep();
    ros::Duration elapsed_time;
    double wrench_ft_frame_flag = 0.0;
    ros::Time start_time = ros::Time::now();
    while (ros::ok())
    {   
        elapsed_time = ros::Time::now() - start_time;

        if (elapsed_time.toSec() >= 2.8)
        {
            break; // 跳出循环
        }
        // std::cout<<wrench_ft_frame[2]<<std::endl;
        TwistStamped.header.stamp = ros::Time::now();
        TwistStamped.twist.linear.x = 0.0;
        TwistStamped.twist.linear.y = 0.0;
        TwistStamped.twist.linear.z = -0.04;
        TwistStamped.twist.angular.x = 0.0;
        TwistStamped.twist.angular.y = 0.0;
        TwistStamped.twist.angular.z = 0.0;
        admit_vel_pub.publish(TwistStamped);
        ros::spinOnce();
        loop_rate.sleep(); 
    }
    std::cout<<wrench_ft_frame[2]<<std::endl;
    if (fabs(wrench_ft_frame[2])>4)
    {
        // 需要搜孔
        flag = 1;
    }

    TwistStamped.header.stamp = ros::Time::now();
    TwistStamped.twist.linear.z = 0.0;
    admit_vel_pub.publish(TwistStamped);
    sleep.sleep();

    changeController("scaled_pos_joint_traj_controller","joint_group_vel_controller"); // 启动-停止
    ROS_INFO("touch over");
}
bool reinforcement::space_arc_node(geometry_msgs::Pose &target_pose)
{   
    double radians = 30 * M_PI / 180.0; // 将角度转换为弧度
    double r = target_pose.position.z - first_node[2] - 0.19;
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose_arc;
    Eigen::Quaterniond quaternion;
    target_pose_arc.orientation = target_pose.orientation;
    Vector4d fz;
    geometry_msgs::TwistStamped TwistStamped;
    ros::Duration sleep(1);
    while (ros::ok() && fabs(wrench_ft_frame[2])>3)
    {   
        get_rotation_matrix(tool_to_base_link,tool_in_base_link,listener,"base","tool0");
        target_pose.position.x = tool_in_base_link[0];
        target_pose.position.y = tool_in_base_link[1];
        target_pose.position.z = tool_in_base_link[2];

        arm.setPoseReferenceFrame("base");
        fz.setZero();
        for (int j = 0; j < 4; j++)
        {   
            quaternion.w() = target_pose.orientation.w;
            quaternion.x() = target_pose.orientation.x;
            quaternion.y() = target_pose.orientation.y;
            quaternion.z() = target_pose.orientation.z;
            ros::Duration(1).sleep();
            ft_bias_client.call(setBias_client);
            switch (j)
            {
                case 0: // x+
                    {Eigen::Quaterniond rotation_y_add(Eigen::AngleAxisd(0.017453293, Eigen::Vector3d::UnitY()));
                    target_pose_arc.position.y = tool_in_base_link[1];
                    for (size_t i = 1; i < 4; i++)
                    {   
                        radians = i * M_PI / 180.0;
                        target_pose_arc.position.x = tool_in_base_link[0] +sin(radians)*r; 
                        target_pose_arc.position.z = tool_in_base_link[2] - (1-cos(radians))*r; 
                        quaternion = rotation_y_add * quaternion;
                        target_pose_arc.orientation.w = quaternion.w();
                        target_pose_arc.orientation.x = quaternion.x();
                        target_pose_arc.orientation.y = quaternion.y();
                        target_pose_arc.orientation.z = quaternion.z();
                        // target_pose_arc.position.x = p_x ;
                        // target_pose_arc.position.z = p_z ;
                        waypoints.push_back(target_pose_arc); 
                        // std::cout<<target_pose_arc<<std::endl;
                    }}
                    break;
                case 1: // x-
                    {target_pose_arc.position.y = tool_in_base_link[1];
                    Eigen::Quaterniond rotation_y_sub(Eigen::AngleAxisd(-0.017453293, Eigen::Vector3d::UnitY()));
                    for (size_t i = 1; i < 4; i++)
                    {   
                        radians = i * M_PI / 180.0;
                        target_pose_arc.position.x = tool_in_base_link[0] -sin(radians)*r; 
                        target_pose_arc.position.z = tool_in_base_link[2] - (1-cos(radians))*r; 
                        quaternion = rotation_y_sub * quaternion;
                        target_pose_arc.orientation.w = quaternion.w();
                        target_pose_arc.orientation.x = quaternion.x();
                        target_pose_arc.orientation.y = quaternion.y();
                        target_pose_arc.orientation.z = quaternion.z();
                        // target_pose_arc.position.x = p_x ;
                        // target_pose_arc.position.z = p_z ;
                        waypoints.push_back(target_pose_arc); 
                        // std::cout<<target_pose_arc<<std::endl;
                    }}
                    break;
                case 2: // y+
                    {target_pose_arc.position.x = tool_in_base_link[0];
                    Eigen::Quaterniond rotation_x_sub(Eigen::AngleAxisd(-0.017453293, Eigen::Vector3d::UnitX()));
                    for (size_t i = 1; i < 4; i++)
                    {   
                        radians = i * M_PI / 180.0;
                        target_pose_arc.position.y = tool_in_base_link[1] +sin(radians)*r; 
                        target_pose_arc.position.z = tool_in_base_link[2] - (1-cos(radians))*r; 
                        quaternion = rotation_x_sub * quaternion;
                        target_pose_arc.orientation.w = quaternion.w();
                        target_pose_arc.orientation.x = quaternion.x();
                        target_pose_arc.orientation.y = quaternion.y();
                        target_pose_arc.orientation.z = quaternion.z();
                        // target_pose_arc.position.y = p_y ;
                        // target_pose_arc.position.z = p_z ;
                        waypoints.push_back(target_pose_arc); 
                        // std::cout<<target_pose_arc<<std::endl;
                    }}
                    break;
                case 3: // y-
                    {target_pose_arc.position.x = tool_in_base_link[0];
                    Eigen::Quaterniond rotation_x_add(Eigen::AngleAxisd(0.017453293, Eigen::Vector3d::UnitX()));
                    for (size_t i = 1; i < 4; i++)
                    {   
                        radians = i * M_PI / 180.0;
                        target_pose_arc.position.y = tool_in_base_link[1] -sin(radians)*r; 
                        target_pose_arc.position.z = tool_in_base_link[2] - (1-cos(radians))*r; 
                        quaternion = rotation_x_add * quaternion;
                        target_pose_arc.orientation.w = quaternion.w();
                        target_pose_arc.orientation.x = quaternion.x();
                        target_pose_arc.orientation.y = quaternion.y();
                        target_pose_arc.orientation.z = quaternion.z();
                        // target_pose_arc.position.y = p_y ;
                        // target_pose_arc.position.z = p_z ;
                        waypoints.push_back(target_pose_arc); 
                        // std::cout<<target_pose_arc<<std::endl;
                    }}
                    break;
                default:
                    break;
            }
            std::cout<<"路点计算完成"<<std::endl;
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.2*0.005;
            arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            robot_trajectory::RobotTrajectory rt(arm.getCurrentState()->getRobotModel(), "manipulator");
            rt.setRobotTrajectoryMsg(*arm.getCurrentState(), trajectory);
            
            trajectory_processing::IterativeParabolicTimeParameterization iptp;
            iptp.computeTimeStamps(rt, 0.05, 1.0);
            rt.getRobotTrajectoryMsg(trajectory);  
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            fixtrajectorytime(trajectory);
            my_plan.trajectory_ = trajectory;
            arm.asyncExecute(my_plan);
            while(mvgr_status.status.status == 3 && ros::ok()){}  // since the topic is to slow, we need to wait till it switches from 3(executed) to 1(in motion)
            while(mvgr_status.status.status != 3 && ros::ok()){fz[j] = wrench_ft_frame[2];} 
            waypoints.clear();
            ros::Duration(1).sleep();

            waypoints.push_back(target_pose);
            moveCartesian(waypoints);
            waypoints.clear();
        }
        std::cout<<fz<<std::endl;
        int minIndex = 4;
        fz.minCoeff(&minIndex);
        std::cout<<minIndex<<std::endl;
        switch (minIndex)
        {
            case 0:
                {   
                    target_pose.position.z += 0.01;
                    waypoints.push_back(target_pose);
                    target_pose.position.x -= 0.004;
                    waypoints.push_back(target_pose);
                    moveCartesian(waypoints);
                    waypoints.clear();
                }
                break;
            case 1:
                {
                    target_pose.position.z += 0.01;
                    waypoints.push_back(target_pose);
                    target_pose.position.x += 0.004;
                    waypoints.push_back(target_pose);
                    moveCartesian(waypoints);
                    waypoints.clear();
                }
                break;
            case 2:
                {   target_pose.position.z += 0.01;
                    waypoints.push_back(target_pose);
                    target_pose.position.y -= 0.004;
                    waypoints.push_back(target_pose);
                    moveCartesian(waypoints);
                    waypoints.clear();
                }
                break;
            case 3:
                {   target_pose.position.z += 0.01;
                    waypoints.push_back(target_pose);
                    target_pose.position.y += 0.004;
                    waypoints.push_back(target_pose);
                    moveCartesian(waypoints);
                    waypoints.clear();
                }
                break;    
            default:
                break;
        }
        sleep.sleep();
        changeController("joint_group_vel_controller","scaled_pos_joint_traj_controller"); // 启动-停止
        ros::Duration elapsed_time;
        double wrench_ft_frame_flag = 0.0;
        ros::Time start_time = ros::Time::now();
        std::cout<<"开始touch时间"<<start_time<<std::endl;
        while (ros::ok())
        {   
            // std::cout<<wrench_ft_frame[2]<<std::endl;
            elapsed_time = ros::Time::now() - start_time;

            if (elapsed_time.toSec() >= 2.7)
            {
                break; // 跳出循环
            }
            TwistStamped.header.stamp = ros::Time::now();
            TwistStamped.twist.linear.x = 0.0;
            TwistStamped.twist.linear.y = 0.0;
            TwistStamped.twist.linear.z = -0.04;
            TwistStamped.twist.angular.x = 0.0;
            TwistStamped.twist.angular.y = 0.0;
            TwistStamped.twist.angular.z = 0.0;
            admit_vel_pub.publish(TwistStamped);
            ros::spinOnce();
            loop_rate.sleep(); 
        }
        TwistStamped.header.stamp = ros::Time::now();
        TwistStamped.twist.linear.z = 0.0;
        admit_vel_pub.publish(TwistStamped);
        sleep.sleep();
        changeController("scaled_pos_joint_traj_controller","joint_group_vel_controller"); // 启动-停止
        std::cout<<"Z轴受力:"<<wrench_ft_frame[2]<<std::endl;
    }
    // 判断是否到孔中心
    if (fabs(wrench_ft_frame[2])<3)
    {   
        std::cout<<"已在孔中心"<<std::endl;
        return true;
    }else
    {
        std::cout<<"未在孔中心"<<std::endl;
        return false;
    }
    
}
void reinforcement::cbf(){
    // ====================================测试CBF=========================
    ros::Duration sleep(1);
    yolov8_ros::ibvs ibvs;
    ros::Publisher  ibvs_v_c_pub = nh_.advertise<geometry_msgs::TwistStamped>("/ibvs_v_c_pub", 1);
    ros::Publisher  ibvs_error_pub = nh_.advertise<geometry_msgs::PointStamped>("/ibvs_error", 1);
    int i = 0;
    std::cout<<"输入3开始IBVS,其他退出"<<std::endl;
    std::cin>>i;
    if (i != 3)
    {
        return;
    }

    geometry_msgs::PointStamped ibvs_error;
    geometry_msgs::TwistStamped TwistStamped;
    std_msgs::Float64MultiArray q_dot_array;
    q_dot_array.data.resize(6);
    std::cout<<"Request to enable ibvs function!"<<std::endl;
    ibvs.request.ibvs_request = true;
    ibvs_srv.call(ibvs);
    sleep.sleep();
    ibvs_start = ibvs.response.ibvs_response;
    sleep.sleep();
    std::cout<<"ibvs_start:"<< ibvs_start <<std::endl;

    changeController("joint_group_vel_controller","scaled_pos_joint_traj_controller"); // 启动-停止
    double convergence_threshold = 0.1;
    bool swich= 1;
    int count = 0;
    double scale = 0.0;
    ros::Time start_time = ros::Time::now();
    while (!ibvs_node.has_converged && ros::ok())
    {   
        ros::spinOnce();
        Eigen::VectorXd q_dot = ibvs_node.ibvs_run(ibvs_data,curr_joint_state);
        if (count!=1000 && scale != 1) // 防止初始启动造成冲击
        {
            scale +=0.001; // 斜率为0.0001的 一次函数
            count++;
        }
        
        ibvs_error.point.x = ibvs_node.error;

        TwistStamped.header.stamp = ros::Time::now();
        ibvs_error.header.stamp = ros::Time::now();
        q_dot_array.data[0] = q_dot[0]* scale;
        q_dot_array.data[1] = q_dot[1]* scale;
        q_dot_array.data[2] = q_dot[2]* scale;
        q_dot_array.data[3] = q_dot[3]* scale;
        q_dot_array.data[4] = q_dot[4]* scale;
        q_dot_array.data[5] = q_dot[5]* scale;
        TwistStamped.twist.linear.x = q_dot[0];
        TwistStamped.twist.linear.y = q_dot[1];
        TwistStamped.twist.linear.z = q_dot[2];
        TwistStamped.twist.angular.x = q_dot[3];
        TwistStamped.twist.angular.y = q_dot[4];
        TwistStamped.twist.angular.z = q_dot[5];
        admit_vel_pub.publish(q_dot_array);
        ibvs_v_c_pub.publish(TwistStamped);
        ibvs_error_pub.publish(ibvs_error);
        // loop_rate.sleep();
        // 避免初始计算时产生的不正确误差
        if (ibvs_node.error > 100  &&  swich== 1)
        {
            convergence_threshold = 0.8;
            swich = 0; 
        }
        
        if (ibvs_node.error < convergence_threshold) 
        {
            ibvs_node.has_converged = true;
            // 速度回0
            TwistStamped.header.stamp = ros::Time::now();
            TwistStamped.twist.linear.x = 0.0;
            TwistStamped.twist.linear.y = 0.0;
            TwistStamped.twist.linear.z = 0.0;
            TwistStamped.twist.angular.x = 0.0;
            TwistStamped.twist.angular.y = 0.0;
            TwistStamped.twist.angular.z = 0.0;
            q_dot_array.data[0] = 0.0;
            q_dot_array.data[1] = 0.0;
            q_dot_array.data[2] = 0.0;
            q_dot_array.data[3] = 0.0;
            q_dot_array.data[4] = 0.0;
            q_dot_array.data[5] = 0.0;
            admit_vel_pub.publish(q_dot_array);
            ibvs_v_c_pub.publish(TwistStamped);
            std::cout<<"伺服完成"<<std::endl;
            // 请求关闭ibvs功能
            ibvs.request.ibvs_request = false;
            ibvs_srv.call(ibvs);
            ibvs_start = ibvs.response.ibvs_response;
            sleep.sleep();                    
        }
    }
    ros::Time end_time = ros::Time::now();
    ros::Duration duration = end_time - start_time;
    ROS_INFO("Elapsed time: %f seconds", duration.toSec());
    changeController("scaled_pos_joint_traj_controller","joint_group_vel_controller"); // 启动-停止

}


void reinforcement::nodeTouch_concret()
{
    geometry_msgs::TwistStamped TwistStamped;
    changeController("joint_group_vel_controller","scaled_pos_joint_traj_controller"); // 启动-停止
    ft_bias_client.call(setBias_client);
    ros::Duration sleep(1);
    sleep.sleep();
    while (ros::ok() && fabs(wrench_ft_frame[2])<3)
    {   
        TwistStamped.header.stamp = ros::Time::now();
        TwistStamped.twist.linear.x = 0.0;
        TwistStamped.twist.linear.y = 0.0;
        TwistStamped.twist.linear.z = -0.04;
        TwistStamped.twist.angular.x = 0.0;
        TwistStamped.twist.angular.y = 0.0;
        TwistStamped.twist.angular.z = 0.0;
        admit_vel_pub.publish(TwistStamped);
        ros::spinOnce();
        loop_rate.sleep(); 
    }

    std::cout<<wrench_ft_frame[2]<<std::endl;
    TwistStamped.header.stamp = ros::Time::now();
    TwistStamped.twist.linear.z = 0.0;
    admit_vel_pub.publish(TwistStamped);
    changeController("scaled_pos_joint_traj_controller","joint_group_vel_controller"); // 启动-停止
    ROS_INFO("touch concrete over");
}