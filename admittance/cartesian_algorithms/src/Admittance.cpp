#include "cartesian_algorithms/admittance/Admittance.h"
// 构造函数 列表初始化赋值
Admittance::Admittance(ros::NodeHandle &n,
    double frequency,
    std::string topic_arm_state,
    std::string topic_arm_command,
    std::string topic_wrench_state,
    std::vector<double> M,
    std::vector<double> D,
    std::vector<double> K,
    std::vector<double> desired_pose,
    std::string base_link,
    std::string end_link,
    double arm_max_vel,
    double arm_max_acc
    ):
    nh_(n),loop_rate_(frequency),
M_(M.data()),D_(D.data()),K_(K.data()),desired_pose_(desired_pose.data()),
arm_max_vel_(arm_max_vel),arm_max_acc_(arm_max_acc),
base_link_(base_link),end_link_(end_link){

    //订阅机械臂状态与腕部（末端）状态
    sub_arm_state_ = nh_.subscribe(topic_arm_state,5,
    &Admittance::state_arm_callback,this,ros::TransportHints().reliable().tcpNoDelay());
    sub_wrench_state_ = nh_.subscribe(topic_wrench_state,5,
    &Admittance::state_wrench_callback,this,ros::TransportHints().reliable().tcpNoDelay());

    // 创建发布者
    pub_arm_cmd_ = nh_.advertise<geometry_msgs::Twist>(topic_arm_command,5);
    pub_arm_cmd_stamp = nh_.advertise<geometry_msgs::TwistStamped>("plot_velocity_cmd",5);
    pub_ee_twist = nh_.advertise<geometry_msgs::TwistStamped>("plot_ee_twist",5);
    pub_ee_pose = nh_.advertise<geometry_msgs::PoseStamped>("plot_ee_pose",5);
    pub_arm_desired_adm_pose = nh_.advertise<geometry_msgs::PoseStamped>("plot_pose_theta",5);
    ft_bias_client = nh_.serviceClient<netft_utils::SetBias>("/bias");

    //传感器偏置初始化
    set_bias.request.toBias = true;
    set_bias.request.forceMax = 30;
    set_bias.request.torqueMax = 10;
    ft_bias_client.call(set_bias);

    // 初始化成员变量
    arm_desired_pose_adm_.setZero(); // 测试变量
    arm_position_.setZero(); //3*1
    arm_twist_.setZero();
    wrench_external_.setZero();//6*1
    desired_pose_position_ <<desired_pose_.topRows(3);
    desired_pose_orientation_.coeffs() << desired_pose_.bottomRows(4)/desired_pose_.bottomRows(4).norm();//四元素归一化

    while (nh_.ok() && !arm_position_(0))// x=0时进入此循环 执行一次回调函数
    {
        ROS_WARN_THROTTLE(1,"waiting for the state of the arm...");//输入警告日志 每秒最多输出一次
        ros::spinOnce();
        loop_rate_.sleep();
    }

    // 初始化速度误差 积分器
    arm_desired_accelaration.setZero();
    arm_desired_twist_adm_.setZero();
    wrench_external_desired.setZero();


    ft_arm_ready_ = false;
    base_world_ready_ = false;
    world_arm_ready_ = false;

    force_x_pre = 0;
    force_y_pre = 0;
    force_z_pre = 0;
    wait_for_transformations();// 坐标变换矩阵
}

void Admittance::wait_for_transformations(){
    tf::TransformListener listener;//变换矩阵监听器
    Matrix6d rot_matrix;
    base_world_ready_ = true;
    world_arm_ready_ = true;
    while (!get_rotation_matrix(rot_matrix, listener, base_link_, end_link_))
    {
     sleep(1);
    }
    ROS_INFO("The Force/Torque sensor is ready to use.");
}

// 控制循环
void Admittance::run(){
    std::vector<double> get_wrench_external_desired;
    ROS_INFO("Running the admittance control loop......");
    while (nh_.ok())
    {   
        if (nh_.getParam("test_desired_pose",test_desired_pose_))
        {
            desired_pose_position_[0] =  test_desired_pose_[0]+0.0001;
            desired_pose_position_[1] =  test_desired_pose_[1]+0.00072;
            desired_pose_position_[2] =  test_desired_pose_[2]+0.21286;
            desired_pose_orientation_.coeffs() << test_desired_pose_[3],test_desired_pose_[4],
            test_desired_pose_[5],test_desired_pose_[6];
        }
        if (!nh_.getParam("twist_scale",twist_scale_))
        {
           twist_scale_ = 0.7;
        }
        nh_.getParam("stiffness_coupling",test_K_);
        K_ = Eigen::Matrix<double,6,6>::Map(test_K_.data());


        if (nh_.getParam("wrench_external_desired",get_wrench_external_desired))
        {
            wrench_external_desired[0] = get_wrench_external_desired[0];
            wrench_external_desired[1] = get_wrench_external_desired[1];
            wrench_external_desired[2] = get_wrench_external_desired[2];
        }

        compute_admittance();

        send_commands_to_robot();

        ros::spinOnce();
        loop_rate_.sleep();
    }
}

void Admittance::compute_admittance()
{   
    Vector6d  test_vel_theta;
    // 通过期望与实际的位姿、外部接触力，以及导纳参数，去计算出此时相应的位置、速度、加速度误差值
    error.topRows(3) = arm_position_ - desired_pose_position_;
    if (desired_pose_orientation_.coeffs().dot(arm_orientation_.coeffs()) < 0 ) // 计算方向是否一致
    {
        arm_orientation_.coeffs() << -arm_orientation_.coeffs();
    }
    Eigen::Quaterniond quat_rot_err(arm_orientation_ * desired_pose_orientation_.inverse()); // 求两者之间的变换矩阵.
    if (quat_rot_err.coeffs().norm() > 1e-3)
    {
        quat_rot_err.coeffs() << quat_rot_err.coeffs()/quat_rot_err.coeffs().norm();
    }
    Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);// 得到其三维旋转空间的一组基，与其旋转角度
    error.bottomRows(3) << err_arm_des_orient.axis()*err_arm_des_orient.angle(); // 四元数转欧拉角？ 至此得到误差的位姿

    Vector6d coupling_wrench_arm;
    if (wrench_external_.segment(0,3).norm()<2  ||  wrench_external_.segment(0,3).norm()>20 )
    {
        wrench_external_.setZero();

    }
    std::cout<<wrench_external_<<std::endl;
    
    coupling_wrench_arm = D_ * (arm_desired_twist_adm_)+ K_ * error ; // 末端误差位姿*k + 末端误差速度*D      
    arm_desired_accelaration = M_.inverse() * (- coupling_wrench_arm + wrench_external_ - wrench_external_desired ); //  M*末端加速度 + D*末端速度 + K*末端位姿 = 外部力 - 期望力（0）


    double a_acc_norm = (arm_desired_accelaration.segment(0,3)).norm(); // 提起前三个元素构成一个三维向量 计算其范数 （位置加速度的范数）
    if (a_acc_norm > arm_max_acc_)
    {
        ROS_WARN_STREAM_THROTTLE(
            1,"Admittance generate high arm accelaration!"
            <<"norm"<<a_acc_norm;
        );
        arm_desired_accelaration.segment(0,3) *= (arm_max_acc_/a_acc_norm); // 放缩至最大量程内
    }
    
    // 积分计算 根据加速度得到 速度值
    ros::Duration duration = loop_rate_.expectedCycleTime(); //得到循环一次的时间 （eg：1s执行100次，即100Hz，得到每次的执行时间0.01s）
    arm_desired_twist_adm_ += arm_desired_accelaration * duration.toSec(); // 离散域的积分 累加：dv/dt = a ----》dv = a*dt  在进行累加，得到期望的末端速度
    arm_desired_pose_adm_ += arm_desired_twist_adm_* duration.toSec();
    // std::cout<<"error:"<<error<<std::endl;
    // std::cout<<"coupling_wrench_arm:"<<coupling_wrench_arm<<std::endl;
    // std::cout<<"a_acc_norm:"<<a_acc_norm<<std::endl;
    // std::cout<<"arm_desired_twist_adm_"<<arm_desired_accelaration<<std::endl;
}

void Admittance::state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr msg){
    arm_position_ << msg->pose.position.x,
                    msg->pose.position.y,
                    msg->pose.position.z;
    
    arm_orientation_.coeffs() << msg->pose.orientation.x,
                                msg->pose.orientation.y,
                                msg->pose.orientation.z,
                                msg->pose.orientation.w;
    ee_pose.header.stamp = ros::Time::now();
    ee_pose.pose.position.x = msg->pose.position.x;
    ee_pose.pose.position.y = msg->pose.position.y;
    ee_pose.pose.position.z = msg->pose.position.z;
    ee_pose.pose.orientation.x = msg->pose.orientation.x;
    ee_pose.pose.orientation.y = msg->pose.orientation.y;
    ee_pose.pose.orientation.z = msg->pose.orientation.z;
    ee_pose.pose.orientation.w = msg->pose.orientation.w;
    
    // 末端速度分量
    arm_twist_ << msg->twist.linear.x,
                msg->twist.linear.y,
                msg->twist.linear.z,
                msg->twist.angular.x,
                msg->twist.angular.y,
                msg->twist.angular.z;
    // std::cout<<msg->pose<<std::endl;
    // std::cout<<"------------------"<<std::endl;
    // std::cout<<msg->twist<<std::endl;

    ee_twist.header.stamp = ros::Time::now();
    ee_twist.twist.linear.x = msg->twist.linear.x;
    ee_twist.twist.linear.y = msg->twist.linear.y;
    ee_twist.twist.linear.z = msg->twist.linear.z;
    ee_twist.twist.angular.x = msg->twist.angular.x;
    ee_twist.twist.angular.y = msg->twist.angular.y;
    ee_twist.twist.angular.z = msg->twist.angular.z;

}
// 获得末端六维力的回调
void Admittance::state_wrench_callback(const geometry_msgs::WrenchStampedConstPtr msg){
    Vector6d wrench_ft_frame;
    Matrix6d rotation_ft_base;
    nh_.getParam("ft_arm_ready",ft_arm_ready_);
    if (ft_arm_ready_) // 力传感器就绪
    {      
        wrench_ft_frame << msg->wrench.force.x,msg->wrench.force.y,msg->wrench.force.z,0.0,0.0,0.0; // 环境坐标系下的力 ,msg->wrench.torque.x,msg->wrench.torque.y,msg->wrench.torque.z
        // float force_low_limit = 50;
        // float force_high_limit = 100;
        
        get_rotation_matrix(rotation_ft_base,listener_ft_,base_link_,end_link_);// 得到末端到baselink的变换矩阵？
        wrench_external_ << rotation_ft_base * wrench_ft_frame * (-1); // 受力需转换到base_link下
    }
    
  
    // if (wrench_external_.norm()>1)
    // {
    //     K_.setZero();
    // }else
    // {
    //     K_ << 10,0,0,0,0,0,
    //           0,10,0,0,0,0,
    //           0,0,10,0,0,0,
    //           0,0,0,10,0,0,
    //           0,0,0,0,10,0,
    //           0,0,0,0,0,10;
    //     std::vector<double> new_desired_pose;
    //     new_desired_pose = {arm_position_(0),arm_position_(1),arm_position_(2),
    //                 arm_orientation_.coeffs()(0), arm_orientation_.coeffs()(1),arm_orientation_.coeffs()(2),arm_orientation_.coeffs()(3)};
    //     nh_.setParam("test_desired_pose",new_desired_pose);
    //     arm_desired_twist_adm_.setZero();
    //     arm_desired_accelaration.setZero();
    // }
}
// 发布速度命令
void Admittance::send_commands_to_robot(){
    
    geometry_msgs::Twist arm_twist_cmd;
    geometry_msgs::TwistStamped arm_twist_cmd_stamped;
    arm_twist_cmd.linear.x = arm_desired_twist_adm_(0) * twist_scale_;
    arm_twist_cmd.linear.y = arm_desired_twist_adm_(1) * twist_scale_;
    arm_twist_cmd.linear.z = arm_desired_twist_adm_(2) * twist_scale_;
    arm_twist_cmd.angular.x = arm_desired_twist_adm_(3) * twist_scale_;
    arm_twist_cmd.angular.y = arm_desired_twist_adm_(4) * twist_scale_;
    arm_twist_cmd.angular.z = arm_desired_twist_adm_(5) * twist_scale_;

    arm_twist_cmd_stamped.twist.linear.x = arm_twist_cmd.linear.x;
    arm_twist_cmd_stamped.twist.linear.y = arm_twist_cmd.linear.y;
    arm_twist_cmd_stamped.twist.linear.z = arm_twist_cmd.linear.z;
    arm_twist_cmd_stamped.twist.angular.x = arm_twist_cmd.angular.x;
    arm_twist_cmd_stamped.twist.angular.y = arm_twist_cmd.angular.y;
    arm_twist_cmd_stamped.twist.angular.z = arm_twist_cmd.angular.z;
    arm_twist_cmd_stamped.header.stamp = ros::Time::now();

    pub_arm_cmd_.publish(arm_twist_cmd);
    // 测试 
    pub_arm_cmd_stamp.publish(arm_twist_cmd_stamped);
    pub_ee_twist.publish(ee_twist);
    pub_ee_pose.publish(ee_pose);
}


// 获取旋转矩阵
bool Admittance::get_rotation_matrix(Matrix6d &rotation_matrix,
    tf::TransformListener &listener,
    std::string from_frame,
    std::string to_frame)
{
    tf::StampedTransform transform;
    Matrix3d rotation_from_to;
    try
    {
        listener.lookupTransform(from_frame,to_frame,ros::Time(0),transform);
        tf::matrixTFToEigen(transform.getBasis(),rotation_from_to); // 获取旋转的部分getBasis
        rotation_matrix.setZero();
        rotation_matrix.topLeftCorner(3,3) = rotation_from_to;
        rotation_matrix.bottomRightCorner(3,3) = rotation_from_to;
    }
    catch(tf::TransformException ex)
    {
        rotation_matrix.setZero();
        ROS_WARN_STREAM_THROTTLE(1,"Wating for tf from: "<< from_frame << "to" << to_frame);// 具有节流功能的警告信息输出，1为两次输出的时间间隔
        return false; 
    }
    return true;
}