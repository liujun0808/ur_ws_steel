#include "cartesian_algorithms/admittance/ibvs.h"
#include <boost/algorithm/string/join.hpp>

ibvs::ibvs():I(height,width),p(4),pd(4),v_c(6),corners_pd(4),corners_p(4),point(4)
{   

    // Get camera extrinsics
    ePc.set(-0.01114759,-0.10696188,-0.13623781,-0.005401,-0.0341412,-0.0199413); // 外参矩阵初始化 ,L_eigen_list(4,Eigen::MatrixXd::Zero(2, 6))
    cam.initPersProjWithDistortion(606.1358642578125,604.9152221679688,317.83746337890625,244.45687866210938,0.0,0.0); // 内参矩阵初始化(fx,fy,u0,v0,kdu,-kdu)(kdu:畸变系数数组)
    
    cdMo.buildFrom(vpTranslationVector(0, 0, 0.218),vpRotationMatrix({1, 0, 0, 0, -1, 0, 0, 0, -1 })); // 期望物体到相机的旋转矩阵
    eMc.buildFrom(ePc);
    // tese 0.218
    // corners_pd[0].set_uv(158, 54);
    // corners_pd[1].set_uv(529, 54);
    // corners_pd[2].set_uv(158, 422);
    // corners_pd[3].set_uv(529, 422); 
    // cbf-ibvs 0.484
    // corners_pd[0].set_uv(263.0, 164.0);
    // corners_pd[1].set_uv(406.0, 164.0);
    // corners_pd[2].set_uv(263.0, 308.0);
    // corners_pd[3].set_uv(406.0, 308.0); 

    // corners_pd[0].set_uv(255.0, 48.0); 
    // corners_pd[1].set_uv(638.0, 48.0);
    // corners_pd[2].set_uv(255.0, 478.0);
    // corners_pd[3].set_uv(638.0, 478.0);
    // 配置视觉伺服任务
    for (size_t i = 0; i < p.size(); i++)
    {
        task.addFeature(p[i],pd[i]);
    }
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(0.5);
    // if (0)
    // {
    //     vpAdaptiveGain lambda(1.5,0.4,30); // 配置自适应增益
    //     task.setLambda(lambda);
    // }else{
    //     task.setLambda(0.5);
    // }
}

geometry_msgs::TwistStamped ibvs::ibvs_run(const std::vector<double> & ibvs_data,const std::vector<vpImagePoint> &corners_pd_new,const double &depth_des,int index) // 使用引用（&）传递参数，避免拷贝，提高效率；cons 不能修改 ibvs_data 的内容
{   // 判断是否识别成功
    geometry_msgs::TwistStamped v_c_msg;
    v_c_msg.twist.angular.x = 0.0;
    v_c_msg.twist.angular.y = 0.0;
    v_c_msg.twist.angular.z = 0.0;
    if ((ibvs_data.back() == -1.0)|| (ibvs_data[11]==0.0)) // 最后一位为标志位，8-11为深度值
    {
        v_c_msg.header.stamp = ros::Time::now();
        v_c_msg.twist.linear.x = 0.0;
        v_c_msg.twist.linear.y = 0.0;
        v_c_msg.twist.linear.z = 0.0;
        return v_c_msg;
    }
    // double z_des = 0.3625;
    // if (corners_pd_new[0].get_i() != 296)
    // {
    //     z_des = ibvs_data[8];
    // }
    // 配置期望特征点坐标
    for (size_t i = 0; i < corners_pd.size(); i++)
    {
        vpFeatureBuilder::create(pd[i],cam,corners_pd_new[i]);
        pd[i].set_Z(depth_des);
    }
    // 获取实时值
    corners_p[0].set_uv(ibvs_data[0],ibvs_data[1]);
    corners_p[1].set_uv(ibvs_data[2],ibvs_data[3]);
    corners_p[2].set_uv(ibvs_data[4],ibvs_data[5]);
    corners_p[3].set_uv(ibvs_data[6],ibvs_data[7]);

    for (size_t i = 0; i < corners_p.size(); i++)
    {
        vpFeatureBuilder::create(p[i],cam,corners_p[i]);
        p[i].set_Z(ibvs_data[i+8]);
    }
    v_c = task.computeControlLaw();
    error = task.getError().sumSquare();

    v_c_msg.header.stamp = ros::Time::now();
    if (index == 0 || index == 1 )
    {
        v_c_msg.twist.linear.x =  v_c[0] ;
        v_c_msg.twist.linear.y =  -v_c[1] ;
        v_c_msg.twist.linear.z =  -v_c[2] ;
    }else
    {
        v_c_msg.twist.linear.x =  v_c[1] ;
        v_c_msg.twist.linear.y =  v_c[0] ;
        v_c_msg.twist.linear.z =  -v_c[2] ;
    }
}

// 建立CBF屏障函数约束


void ibvs::conmupt_jacobi(const std::vector<double>& joint_pos)
{
    // 加载机器人模型
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description","ur10_robot");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    const robot_model::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    // // 获取关节组
    // const std::vector<std::string>& joint_names = joint_model_group->getJointModelNames();
    // std::cout << "MoveIt Joint Order:\n";
    // for (const auto& name : joint_names) {
    //     std::cout << " - " << name << std::endl;
    // }
    kinematic_state->setJointGroupPositions(joint_model_group, joint_pos);
    // 计算雅可比矩阵
    Eigen::Vector3d reference_point(0.0, 0.0, 0.0); // xyz="-0.0001 -0.00072 0.21286"
    // Eigen::Vector3d reference_point(-0.0001, -0.000072, 0.21286); 
    kinematic_state->getJacobian(joint_model_group,
                                kinematic_state->getLinkModel("tool0"),
                                reference_point,
                                jacobian);
}

ibvs::~ibvs()
{}