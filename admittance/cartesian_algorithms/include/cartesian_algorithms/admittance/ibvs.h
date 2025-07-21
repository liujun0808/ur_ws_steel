
#ifndef IBVS_H
#define IBVS_H
// #ifndef VISP_HAVE_REALSENSE2
//   #if defined(VISP_CXX_STANDARD) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)

#include <iostream>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
// #include <visp3/robot/vpRobotUniversalRobots.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <osqp/osqp.h>
#include <ros/ros.h>
#include "geometry_msgs/WrenchStamped.h"


class qp_solve
{
private:
    /* data */
    Eigen::SparseMatrix<double> H; // 构造目标函数
    Eigen::SparseMatrix<double> A ; // 约束
    Eigen::VectorXd g,l,u;
    OSQPCscMatrix* eigenSparseToCsc(const Eigen::SparseMatrix<double>& mat);
    OSQPCscMatrix* H_csc;
    OSQPCscMatrix* A_csc;
    OSQPSolver* solver;
    OSQPSettings* settings;
    Eigen::VectorXd theta_n; // 标准控制输入 
    

public:
    qp_solve();
    Eigen::VectorXd solveQP(const Eigen::MatrixXd& A_constraints, 
                            const Eigen::VectorXd& b_constraints,
                            const Eigen::VectorXd& theta_n);
    ~qp_solve();
};

qp_solve::qp_solve():H(6,6)
{
    H = 2 * Eigen::MatrixXd::Identity(6, 6).sparseView();  // OSQP要求目标函数为0.5θ^T H θ + g^T θ，因此H需乘以2
    
    H_csc= nullptr;
    A_csc = nullptr;
    solver = nullptr;
    settings = (OSQPSettings*)malloc(sizeof(OSQPSettings)); //配置OSQP求解器
    osqp_set_default_settings(settings);
    settings->linsys_solver = OSQP_DIRECT_SOLVER;
    settings->verbose = false; // 关闭详细输出
    settings->eps_abs = 1e-7; // 提高求解精度
    settings->eps_rel = 1e-7;
    settings->max_iter = 10000;      // 增加最大迭代次数
    // settings->adaptive_rho = 1;     // 启用自适应步长


}
Eigen::VectorXd qp_solve::solveQP(const Eigen::MatrixXd& A_constraints, 
                                const Eigen::VectorXd& b_constraints,
                                const Eigen::VectorXd& theta_n)
{   

    g = -2.0 * theta_n; // theta_n为名义控制输入
    A = A_constraints.sparseView();
    l = b_constraints;
    Eigen::VectorXd u = Eigen::VectorXd::Constant(A.rows(), OSQP_INFTY);
    Eigen::SparseMatrix<double> H_upper = H.triangularView<Eigen::Upper>();
    H_csc = eigenSparseToCsc(H_upper);
    A_csc = eigenSparseToCsc(A);
    // std::cout<<"QPing...."<<std::endl;
    // 直接传递参数，无需OSQPData
    osqp_setup(&solver, 
              H_csc,      // 目标函数矩阵P（即H）
              g.data(),   // 目标函数线性项g
              A_csc,      // 约束矩阵A
              l.data(),   // 约束下界l
              u.data(),   // 约束上界u
              A.rows(),   // 约束数m
              6,          // 变量数n
              settings);

    osqp_solve(solver); // 求解结果

    // 步骤6：提取解并处理结果
    Eigen::VectorXd theta_opt(6);
    if (solver->info->status_val == OSQP_SOLVED) {
        for (int i = 0; i < 6; ++i) {
            theta_opt(i) = solver->solution->x[i];
        }
    } else {
        ROS_WARN("OSQP求解失败!使用标准控制输入。");
        theta_opt = theta_n;
    }
    osqp_cleanup(solver);
    free(H_csc->p); free(H_csc->i); free(H_csc->x); free(H_csc);
    free(A_csc->p); free(A_csc->i); free(A_csc->x); free(A_csc);
    // free(settings);
    // std::cout<<"QPending...."<<std::endl;
    return theta_opt;
}

// OSQPCscMatrix 是 OSQP 需要的 CSC 格式的矩阵结构，它通常包含：
// nzmax：非零元素的数量（最大存储空间）。
// m：行数。
// n：列数。
// p：列指针数组，存储每列的起始索引。
// i：行索引数组，存储非零元素所在的行索引。
// x：非零元素数组，存储矩阵的值。
OSQPCscMatrix* qp_solve::eigenSparseToCsc(const Eigen::SparseMatrix<double>& mat)
{
    OSQPCscMatrix* csc = (OSQPCscMatrix*)malloc(sizeof(OSQPCscMatrix)); // 开辟一块结构体内存返回给指针
    csc->nzmax = mat.nonZeros(); // 记录非零元素个数
    csc->m = mat.rows();
    csc->n = mat.cols();
    csc->p = (OSQPInt*)malloc((csc->n + 1) * sizeof(OSQPInt));
    csc->i = (OSQPInt*)malloc(csc->nzmax * sizeof(OSQPInt));
    csc->x = (OSQPFloat*)malloc(csc->nzmax * sizeof(OSQPFloat));
    std::copy(mat.outerIndexPtr(), mat.outerIndexPtr() + mat.outerSize() + 1, csc->p); // 复制 Eigen 的稀疏矩阵的列指针（column pointers） 到 OSQP 的 CSC 结构。
    // 填充行索引（i）和值（x）
    int index = 0;
    for (int col = 0; col < mat.outerSize(); ++col) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(mat, col); it; ++it) {
            csc->i[index] = it.row();
            csc->x[index] = it.value();
            index++;
        }
    }
    return csc;
}

qp_solve::~qp_solve()
{
    free(settings);

}

class ibvs
{
private:

    unsigned int width = 640, height = 480;  
    vpPoseVector ePc; // xyz rx ry rz
    vpHomogeneousMatrix eMc; // 转为齐次矩阵
    // Get camera intrinsics 
    vpCameraParameters cam;
    vpImage<unsigned char> I;
    vpHomogeneousMatrix cdMc, cMo, oMo; // 相机期望位姿，当前物体到相机的旋转矩阵，物体的位姿？
    vpHomogeneousMatrix cdMo; // 期望物体到相机的旋转矩阵
    vpServo task; // 视觉伺服任务
    std::vector<vpFeaturePoint> p,pd; // 定义特征点、期望特征点
    vpColVector v_c; // 速度控制向量
    std::vector<vpImagePoint>  corners_pd; // 期望特征角点像素坐标
    std::vector<vpImagePoint> corners_p;    // 实时特征角点像素坐标

    bool final_quit = false;  // 定义标志位
    bool send_velocities = false;
    bool servo_started = false;
    std::vector<vpPoint> point;
    
    // 增加了CBF
    Eigen::MatrixXd K_s,RCT,RCT_block;
    Eigen::MatrixXd A; // 约束矩阵
    Eigen::VectorXd b;
    double gamma;
    std::vector<double> qmin,qmax;
    vpMatrix L_all; // 交互矩阵
    std::vector<Eigen::MatrixXd> L_eigen_list; // 交互矩阵容器
    Eigen::VectorXd v_c_,qn_dot,qn_dot_new,qn_dot_newPre,qn_dot_newFliter;
    ros::NodeHandle cbf;
    ros::Publisher qn_new_pub;
    geometry_msgs::WrenchStamped qn_new_msg;



public:
    ibvs(/* args */);
    ~ibvs();
    double error = 20;
    bool has_converged = false;
    qp_solve qp_solve_;
    Eigen::MatrixXd jacobian;
    
    void conmupt_jacobi(const std::vector<double>& joint_pos);
    //主运行函数 
    Eigen::VectorXd ibvs_run(const std::vector<double> & ibvs_data,const std::vector<double>& joint_pos);

};
//   #endif
// #endif



#endif
