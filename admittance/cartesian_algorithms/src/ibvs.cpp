#include "cartesian_algorithms/admittance/ibvs.h"
#include <boost/algorithm/string/join.hpp>

ibvs::ibvs():I(height,width),p(4),pd(4),v_c(6),corners_pd(4),v_c_(6),qn_dot(6),qn_dot_new(6)
,corners_p(4),point(4),L_all(8,6),A(10,6),b(10),RCT(6,6),RCT_block(3,3),qn_dot_newPre(6),qn_dot_newFliter(6) 
{   

    // Get camera extrinsics
    ePc.set(-0.01114759,-0.10696188,-0.13623781,-0.005401,-0.0341412,-0.0199413); // 外参矩阵初始化 ,L_eigen_list(4,Eigen::MatrixXd::Zero(2, 6))
    cam.initPersProjWithDistortion(606.1358642578125,604.9152221679688,317.83746337890625,244.45687866210938,0.0,0.0); // 内参矩阵初始化(fx,fy,u0,v0,kdu,-kdu)(kdu:畸变系数数组)
    
    cdMo.buildFrom(vpTranslationVector(0, 0, 0.218),vpRotationMatrix({1, 0, 0, 0, -1, 0, 0, 0, -1 })); // 期望物体到相机的旋转矩阵
    eMc.buildFrom(ePc);
    // tese 0.218
    corners_pd[0].set_uv(158, 54);
    corners_pd[1].set_uv(529, 54);
    corners_pd[2].set_uv(158, 422);
    corners_pd[3].set_uv(529, 422); 
    // cbf-ibvs 0.484
    // corners_pd[0].set_uv(263.0, 164.0);
    // corners_pd[1].set_uv(406.0, 164.0);
    // corners_pd[2].set_uv(263.0, 308.0);
    // corners_pd[3].set_uv(406.0, 308.0); 

    // corners_pd[0].set_uv(255.0, 48.0); 
    // corners_pd[1].set_uv(638.0, 48.0);
    // corners_pd[2].set_uv(255.0, 478.0);
    // corners_pd[3].set_uv(638.0, 478.0);
    for (size_t i = 0; i < corners_pd.size(); i++)
    {
        vpFeatureBuilder::create(pd[i],cam,corners_pd[i]);
        pd[i].set_Z(0.218);
        // pd[i].set_Z(0.188);
        // std::cout <<"Pd";
        // std::cout <<pd[i].get_x()<< " ";
        // std::cout <<pd[i].get_y()<< " ";
        // std::cout <<pd[i].get_Z()<< "\n";
    }

    // 初始化参数 增加CBF后
    jacobian.setZero();
    A.setZero();
    b.setZero();
    gamma = 5;
    qmin.assign({-3.14, -2.36, -3.14, -3.14, -3.14, -3.14});
    qmax.assign({3.14, 2.36, 3.14,  3.14, 3.14, 3.14});

    // qmin.assign({-6.28, -6.28, -3.14, -6.28, -6.28, -6.28});
    // qmax.assign({6.28, 6.28, 3.14,  6.28, 6.28, 6.28});
    // RCT_block<<0.99921847,0.02002815,-0.03407832,
    //             -0.01984378,0.99978662,0.0057399,
    //             0.03418601,-0.00505917,0.99940268;

    RCT_block<<-1.0, 0.0, 0.0,
                0.0, 1.0,  0.0,
                0.0, 0.0, -1.0;
    RCT.setZero();
    RCT.block(0,0,3,3) = RCT_block.transpose();
    RCT.block(3,3,3,3) = RCT_block.transpose();
    L_eigen_list.resize(4);
    for (int i = 0; i < L_eigen_list.size(); ++i) {
        L_eigen_list[i] = Eigen::MatrixXd(2, 6);  // 每个矩阵的大小是 2x6
    }
    qn_dot_newPre.setZero();
    qn_dot_newFliter.setZero();
    qn_new_pub = cbf.advertise<geometry_msgs::WrenchStamped>("/qn_new",1);

}

Eigen::VectorXd ibvs::ibvs_run(const std::vector<double> & ibvs_data,const std::vector<double>& joint_pos) // 使用引用（&）传递参数，避免拷贝，提高效率；cons 不能修改 ibvs_data 的内容
{   // 判断是否识别成功
    if ((ibvs_data.back() == -1.0)|| (ibvs_data[11]==0.0))
    {
        // code
        // std::cout<<"ibvs_data数据异常"<<std::endl;
        qn_dot.setZero();
        return qn_dot;
    }
    
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

    // 指的是返回自 1970 年 1 月 1 日 00:00:00 UTC（协调世界时） 以来经过的时间，以 毫秒（milliseconds） 计算。
    // static double t_init_servo = vpTime::measureTimeMs();
    // double t_start = vpTime::measureTimeMs();

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
    for (int i = 0; i < v_c.getRows(); ++i) {
        v_c_[i] = v_c[i];  // 将vpColVector v_c转换为eigen
        // std::cout<<v_c_[i]<<" ";
    }
    v_c_[3] = 0.0;
    v_c_[4] = 0.0;
    v_c_[5] = 0.0;
    conmupt_jacobi(joint_pos);
    qn_dot = jacobian.inverse() * RCT * v_c_; // 计算标准控制输入

    error = task.getError().sumSquare();
    // return qn_dot;
    // CBF++++++++++++++++++++++++++++++++++++++++
    //计算交互矩阵
    L_all = task.computeInteractionMatrix();
    
    // std::cout<<"L_all"<<std::endl<<L_all<<std::endl;

    for (int i = 0; i < L_all.getRows(); i+=2) {
        // 提取第k个特征点的交互矩阵块
        Eigen::MatrixXd block(2, L_all.getCols());
        for (size_t j = 0; j < L_all.getCols(); j++) {
                block(0, j) = L_all[i][j];     // 取 i 行
                block(1, j) = L_all[i + 1][j]; // 取 i+1 行
            }
        // std::cout<<"block"<<std::endl<<block<<std::endl;
        switch (i)
        {
        case 0:
            L_eigen_list[0] = block;
            break;
        case 2:
            L_eigen_list[1] = block;
            break;
        case 4:
            L_eigen_list[2] = block;
            break;
        case 6:
            L_eigen_list[3] = block;
            break;
        default:
            break;
        }
    }
    // std::cout<<"L_eigen_list 计算结束"<<std::endl;
    // for (size_t i = 0; i < 4; i++)
    // {
    //     /* code */
    //     std::cout<<"L_eigen_list"<<i<<std::endl<<L_eigen_list[i]<<std::endl;
    // }

    // 关节限制约束（式10-11）
    for (int i = 0; i < 6; ++i) {
        double q_i = joint_pos[i];
        double h_lim = (qmax[i] - q_i) * (q_i - qmin[i]) / pow(qmax[i] - qmin[i], 2);
        double dh_lim = (qmax[i] + qmin[i] - 2*q_i) / pow(qmax[i] - qmin[i], 2);
        A(i,i) = dh_lim; // 假设每个关节独立
        b(i) = -gamma * h_lim;
    }
    // 视野约束（式7-8）
    std::vector<double> distanse_s(4);// 距离存储
    Eigen::Vector2d s_e(0,0);//当前点的距离最近点
    Eigen::Vector2d s(0,0);//当前点
    vpFeaturePoint temp_se;
    vpImagePoint temp_;
    for (int i = 0; i < 4; ++i) {
        s<<corners_p[i].get_u(),corners_p[i].get_v();
        // 计算每个点距离约束框最近的距离
        distanse_s = {pow(corners_p[i].get_v(),2), 
                        pow(corners_p[i].get_v()-480,2),
                        pow(corners_p[i].get_u(),2), 
                        pow(corners_p[i].get_u()-640,2)};//计算当前特征点与四边界框的距离，取最小值

        auto dis_min_it = std::min_element(distanse_s.begin(),distanse_s.end());
        // double  dis_min = *dis_min_it;
        // if (dis_min >= 100 ) {
        //     // 特征点已在安全区域内，无需像素范围约束
        //     A.row(6+i).setZero();
        //     b(6+i) = 0;
        // } else {
            int min_index = std::distance(distanse_s.begin(), dis_min_it);  // 计算索引
            switch (min_index)
            {
            case  0:
                temp_.set_uv(corners_p[i].get_u(), 80);
                vpFeatureBuilder::create(temp_se, cam, temp_);
                s_e(0) = temp_se.get_x();
                s_e(1) = temp_se.get_y();
                s(0) = p[i].get_x();
                s(1) = p[i].get_y();
                break;
            case  1:
                // s_e<<corners_p[i].get_u(), 470;
                temp_.set_uv(corners_p[i].get_u(), 400);
                vpFeatureBuilder::create(temp_se, cam, temp_);
                s_e(0) = temp_se.get_x();
                s_e(1) = temp_se.get_y();
                s(0) = p[i].get_x();
                s(1) = p[i].get_y();
                break;
            case  2:
                // s_e<<10, corners_p[i].get_v();
                temp_.set_uv(80, corners_p[i].get_v());
                vpFeatureBuilder::create(temp_se, cam, temp_);
                s_e(0) = temp_se.get_x();
                s_e(1) = temp_se.get_y();
                s(0) = p[i].get_x();
                s(1) = p[i].get_y();
                break;
            case  3:
                // s_e<<630, corners_p[i].get_v();
                temp_.set_uv(560, corners_p[i].get_v());
                vpFeatureBuilder::create(temp_se, cam, temp_);
                s_e(0) = temp_se.get_x();
                s_e(1) = temp_se.get_y();
                s(0) = p[i].get_x();
                s(1) = p[i].get_y();
                break;
            default:
                s_e = s;
                break;
            }

            Eigen::MatrixXd ds_dq(2,6); 
            // std::cout<<"L_eigen_list"<<L_eigen_list[i]<<std::endl;
            ds_dq = L_eigen_list[i] * RCT * jacobian; // 根据式8-9计算导数
            // std::cout<<"ds_dq"<<ds_dq<<std::endl;

            A.row(6+i) = 2*(s - s_e).transpose() * ds_dq;
            b(6+i) = -gamma * (-0.5 * (s - s_e).squaredNorm());
            // std::cout<<"A:"<<A<<std::endl;
            // std::cout<<"B:"<<b<<std::endl;
            // std::cout<<i<<"  s:"<<s.transpose()<<"  s_e:"<<s_e.transpose()<<std::endl;

        // }
    }
    // 二次规划求解    
    qn_dot_new = qp_solve_.solveQP(A,b,qn_dot);
    // qn_dot_newFliter = 0.4 * qn_dot_newPre + 0.6 * qn_dot_new; // 低通滤波
    // qn_dot_newPre = qn_dot_newFliter;

    // std::cout<<"qn_dot"<<qn_dot.transpose()<<std::endl;
    // std::cout<<"qn_dot_new"<<qn_dot_new.transpose()<<std::endl;

    // if ((b(6)+b(7)!=0))
    // {
    //     int i = 0;
    //     std::cout<<"输入4使用qp结果,否则使用标称输出"<<std::endl;
    //     std::cin>>i;
    //     if (i == 4)
    //     {
    //     }
    //     else{
    //         qn_dot_new = qn_dot;
    //     }
    // }
    A.setZero();
    b.setZero();

    qn_new_msg.header.stamp = ros::Time::now();
    qn_new_msg.wrench.force.x = qn_dot_new[0];
    qn_new_msg.wrench.force.y = qn_dot_new[1];
    qn_new_msg.wrench.force.z = qn_dot_new[2];
    qn_new_msg.wrench.torque.x = qn_dot_new[3];
    qn_new_msg.wrench.torque.y = qn_dot_new[4];
    qn_new_msg.wrench.torque.z = qn_dot_new[5];
    qn_new_pub.publish(qn_new_msg);

    // CBF++++++++++++++++++++++++++++++++++++++++
    return qn_dot;
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