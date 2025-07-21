#include <pluginlib/class_list_macros.h>
#include "cartesian_position_controller/cartesian_position_controller.h"
#include "cartesian_position_controller/kinematics_base.h"
#include "kdl_conversions/kdl_msg.h"

namespace cartesian_position_controller
{
    // 初始化运动学链 为后续运动学求解
    bool Cartesian_Position_Controller::init(   //ros_control中约定的函数 init start update stop...
        hardware_interface::PositionJointInterface *robot,ros::NodeHandle &n) //硬件抽象为软件资源)
    {
        // KDL
        kinematics_base::Kinematics_Base::init(robot,n);
        // new 创建动态内存区 再用智能指针接收
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv_givens(this->kdl_chain_));//ik速度求解器
        ik_pos_solver_.reset(new KDL::ChainIkSolverPos_LMA(this->kdl_chain_));//ik位置求解器
        fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(this->kdl_chain_));//fk速度求解器
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(this->kdl_chain_));//fk位置求解器 recursive 递归

        // 获得发布周期参数
        if (!n.getParam("publish_rate",publish_rate_))
        {
            ROS_INFO("Parameter 'publish_rate' not set");
            return false;
        }
        // 动态创建实时发布者 用智能指针接收  
        realtime_pub_.reset(new realtime_tools::RealtimePublisher<cartesian_state_msgs::PoseTwist>(n,"ee_state",4));

        // 订阅阻抗算法发布的命令话题 使用TCP无延时算法
        sub_command_ = n.subscribe("command_car_pos",5,
            &Cartesian_Position_Controller::command_cart_pos,this,
            ros::TransportHints().reliable().tcpNoDelay());
        
        // 变量初始化,初始化关节数组的大小为chain中的关节数量
        this->joint_state_.resize(this->kdl_chain_.getNrOfJoints());
        this->joint_effort_.resize(this->kdl_chain_.getNrOfSegments());
        Jnt_Vel_Cmd_.resize(this->kdl_chain_.getNrOfJoints());
        Jnt_Vel_Cmd_.resize(this->kdl_chain_.getNrOfJoints());

        End_Vel_Cmd_ = KDL::Twist::Zero();//赋初值
        End_Pos_.p.Zero();//位置
        End_Pos_.M.Identity();//姿态单位化
        End_Vel_.p.Zero();
        End_Vel_.M.Identity();
        pre_e = 0.0;
        return true; 
    }

    // start函数
    void Cartesian_Position_Controller::starting(const ros::Time &time){
        // 根据关节数量，对变量赋初值
        for(std::size_t i =0;i<joint_handles_.size();i++)
        {   //KDL求解器需要的参数赋初值
            Jnt_Vel_Cmd_(i) = 0.0;
            Jnt_Pos_Cmd_(i) = 0.0;
            this->joint_state_.q(i) = 0.0; //机械臂关节状态
            this->joint_state_.qdot(i) = 0.0;
            this->joint_effort_(i) = 0.0;
            this->joint_state_.q(i) = this->joint_handles_[i].getPosition();//获得每个关节状态的位置信息
        }
        End_Vel_Cmd_ = KDL::Twist::Zero();//末端速度命令赋初值
        fk_pos_solver_->JntToCart(this->joint_state_.q,End_Pos_Cmd_);//使用求解器 根据关节位姿求出末端位姿
        last_publish_time_ = time;//更新发布时间
    }

    void Cartesian_Position_Controller::update(const ros::Time &time, const ros::Duration &period){
        //获取每个关机的位姿 速度 力
        for (size_t i = 0; i < joint_handles_.size(); i++)
        {
            this->joint_state_.q(i) = this->joint_handles_[i].getPosition();
            this->joint_state_.qdot(i) = this->joint_handles_[i].getVelocity();
            this->joint_effort_(i) = this->joint_handles_[i].getEffort();
        }
        // 根据运动学求解 用输入当前关节位置 ，期望末端位置、速度 输出关节位置、速度指令
        ik_vel_solver_->CartToJnt(this->joint_state_.q,End_Vel_Cmd_,Jnt_Vel_Cmd_);
        ik_pos_solver_->CartToJnt(this->joint_state_.q,End_Pos_Cmd_,Jnt_Pos_Cmd_);
        writePositionCommands(period); //写关节位置命令 ，Jnt_Pos_Cmd_,去动作各关节
        
        //写完后进行 正向运动学计算,得到末端的位姿与速度， （为什么不是根据动作完后的最新状态去计算？）
        fk_pos_solver_->JntToCart(this->joint_state_.q,End_Pos_);
        fk_vel_solver_->JntToCart(this->joint_state_,End_Vel_);

        // 限制发布频率 
        if (publish_rate_ > 0.0 && last_publish_time_+ros::Duration(1.0/publish_rate_) < time) //上一个发布时间+发布周期后 小于当前时间 则可以继续发布
        {
            // 发布消息
            if (realtime_pub_->trylock()) // 获取数据锁，得到实时发布的权限
            {
                last_publish_time_ += ros::Duration(1.0/publish_rate_);// 更新上一次发布的时间
                realtime_pub_->msg_.header.stamp = time; // 时间戳赋值
                tf::poseKDLToMsg(End_Pos_,realtime_pub_->msg_.pose); // 转换为msg类型消息
                tf::twistKDLToMsg(End_Vel_.GetTwist(),realtime_pub_->msg_.twist);

                realtime_pub_->unlockAndPublish();
            }
        }
    }

    // 回调函数，获取阻抗算法计算的结果
    void Cartesian_Position_Controller::command_cart_pos(const geometry_msgs::PoseConstPtr &msg){ // 指针常量const分别表示 指向的对象引用msg不可修改；指向对象的指针是不可修改的
        double x,y,z,w;
        x = msg->orientation.x;
        y = msg->orientation.y;
        z = msg->orientation.z;
        w = msg->orientation.w;

        End_Pos_Vector[0] = msg->position.x;
        End_Pos_Vector[1] = msg->position.y;
        End_Pos_Vector[2] = msg->position.z;
        // 获取消息
        End_Pos_Cmd_.p = End_Pos_Vector;
        End_Pos_Cmd_.M = End_Pos_Rotation.Quaternion(x,y,z,w);
    }
    void Cartesian_Position_Controller::writeVelocityCommands(const ros::Duration &period){
        for (size_t i = 0; i < joint_handles_.size(); i++)
        {
            this->joint_handles_[i].setCommand(Jnt_Vel_Cmd_(i));
        }
    }
    void Cartesian_Position_Controller::writePositionCommands(const ros::Duration &period){
        for (size_t i = 0; i < joint_handles_.size(); i++)
        {
            double pos_detla;
            double Kp = 0.05;
            double Kd = 0.00;
            this->joint_handles_[i].setCommand(Jnt_Pos_Cmd_(i));
        }
        
    }
}
// 注册控制器插件，可由control manager 载入使用 （注册插件的类名，继承的基类名）
PLUGINLIB_EXPORT_CLASS(cartesian_position_controller::Cartesian_Position_Controller,controller_interface::ControllerBase)