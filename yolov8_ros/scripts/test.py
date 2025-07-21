import os
import sys
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose,PoseStamped
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf
from yolov8_ros.msg import objectArray,workpieces,grasp_flag
from yolov8_ros.srv import center,centerRequest,centerResponse
from std_msgs.msg import Bool


# 导入自定义模块时需要设置该模块的路径（默认是从该工作空间开始寻找）
path = os.path.abspath(".") # 获取执行终端下该目录的路径
sys.path.insert(0,path+"/src/yolov8_ros/scripts") # 在0行处动态插入该路径

from jodell_gripper import jodell_gripper

class moveitCartesianDemo():
    def __init__(self) -> None:
        self.gripper = jodell_gripper()
        self.gripper.open_gripper()
        rospy.init_node('moveit')
        
        self.listener = tf.TransformListener()
        rospy.sleep(1)
        # 载入矩阵参数
        self.cam_pose = np.loadtxt('/home/lj/project/ur_ws/src/yolov8_ros/include/camera_pose.txt', delimiter=' ') # 外参矩阵
        self.cam_depth_scale = np.loadtxt('/home/lj/project/ur_ws/src/yolov8_ros/include/camera_depth_scale.txt', delimiter=' ')
        self.cam_k = np.loadtxt('/home/lj/project/ur_ws/src/yolov8_ros/include/camera_k_640_480.txt', delimiter=' ') # 内参矩阵
        # 初始化规划组
        self.arm = MoveGroupCommander('manipulator')
        # 允许规划失败后重新规划
        self.arm.allow_replanning(True)

        # 设置目标位姿参考坐标系
        self.arm.set_pose_reference_frame('base')
     
        # 设置位置(m)与姿态的允许误差(rad)
        self.arm.set_goal_position_tolerance(0.002)
        self.arm.set_goal_orientation_tolerance(0.002)

        # 设置允许的最大速度和加速度系数
        self.arm.set_max_velocity_scaling_factor(0.5)
        self.arm.set_max_acceleration_scaling_factor(0.5)

        # 获取末端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()
        
        #订阅工件消息
        self.sub_workpieces = rospy.Subscriber("workpieces",objectArray,self.callBack,queue_size=1)
        rospy.sleep(3)
        self.grasping_pub = rospy.Publisher('grasping',grasp_flag,queue_size=1)
        self.client = rospy.ServiceProxy("center",center) # 创建客户端
        self.node_state_pub = rospy.Publisher('node_state',Bool,queue_size=1) # 创建节点状态发布者
        # 抓取标志
        self.grasping = grasp_flag()
        self.grasping.flag = False # 初始化为false

        # 创建工件分类器
        self.nodes = objectArray()
        self.verticals =  objectArray()
        self.horizontals = objectArray()

        # 节点与杆件放置的位置坐标初始化
        self.position_dic = {
            'node0':[0,0,0],'node1':[0,0,0],'node2':[0,0,0],'node3':[0,0,0],
            'ver0':[0,0,0],'ver1':[0,0,0],'ver2':[0,0,0],'ver3':[0,0,0],
            'hori0':[0,0,0],'hori1':[0,0,0],'hori2':[0,0,0],'hori3':[0,0,0]
        } 
        # home状态下检测的每个节点位于base下的坐标用于二次拍照
        self.node_position_first = []
        # 第二层第一个节点放置的坐标
        self.second_1node= []


    # 控制机械臂回到home
    def goHome(self):   
        self.arm.set_named_target('home')
        self.arm.go(wait=True)
        rospy.sleep(1)

    # 获取当前tool0 位于base的位姿
    def getPose(self):
        target_frame = 'base'
        source_frame = 'tool0'
        try:
            self.listener.waitForTransform(target_frame,source_frame,rospy.Time(0),rospy.Duration(1))
            (trans1, rot1) = self.listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            # print("Translation: ", trans1)
            # print("Rotation: ", rot1) # x,y,z,w
        except:
            rospy.logerr("Failed to get transform between %s and %s", target_frame, source_frame)
        return trans1,rot1
    
    # 获取当前camera_color_frame 位于base的位姿
    def getPose_camera(self):
        target_frame = 'base'
        source_frame = 'camera_color_frame'
        try:
            self.listener.waitForTransform(target_frame,source_frame,rospy.Time(0),rospy.Duration(1))
            (trans1, rot1) = self.listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            # print("相机在base下: ", trans1)
            # print("Rotation: ", rot1) # x,y,z,w
        except:
            rospy.logerr("Failed to get transform between %s and %s", target_frame, source_frame)
        return trans1,rot1  
    
    def getPosition(self):
        target_frame = 'base'
        source_frame = 'tool0'
        try:
            self.listener.waitForTransform(target_frame,source_frame,rospy.Time(0),rospy.Duration(1))
            (trans1, rot1) = self.listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            # print("Translation: ", trans1)
            # print("Rotation: ", rot1) # x,y,z,w
        except:
            rospy.logerr("Failed to get transform between %s and %s", target_frame, source_frame)
        return trans1
    
    # 第一层横向筋
    def first_path_horizontal(self,hori):
        (position,orientation) = self.getPose() # 基于base
        waypoints = []  # 初始化路径列表
        # 计算平均深度值 规避深度值为0的点
        depth_avg = 0.0
        count = 0.0
        if len(hori)!=8:
            print('识别数量有误：',len(hori))
            return
        for i in hori:
            if i.depth !=0:
                depth_avg +=i.depth
                count +=1
        if count !=0:
            depth_avg /= count
        else:
            print('4个深度均为0')
            return False
        print('平均深度为：',depth_avg)
        # 用平均值替代深度为0的点
        for i in hori:
            if i.depth ==0:
                i.depth = depth_avg

        for i in range(0,4):
            print('抓取第{}个工件'.format(i+1))

            point = np.array([hori[i].center_x,hori[i].center_y,693],dtype=object)
            point_ = self.camera2base(point)
            # 写死z方向值
            point_[2] = 0.034
            print('要抓取的点:',point_)

            start_pose = Pose() 
            # 内容替换，保留格式
            start_pose.orientation.x = float(orientation[0])
            start_pose.orientation.y = float(orientation[1])
            start_pose.orientation.z = float(orientation[2])
            start_pose.orientation.w = float(orientation[3])
            # 抓取时补偿
            comp_z = 0.0
            comp_y = 0.0
            comp_x = 0.0
            if i==0:
                comp_y = 0.00
            if i ==1:
                comp_y = -0.002
            if i ==2:
                comp_y = -0.001
            if i==3:
                comp_y = -0.002


            # 初始化路径列表
            # waypoints.append(start_pose)
            
            # *******抓取******
            wp = deepcopy(start_pose)
            wp.position.x = float(point_[0])
            wp.position.y = float(point_[1])+comp_y # 存在误差需补偿
            wp.position.z = float(point_[2]) # 误差补偿
            waypoints.append(deepcopy(wp)) # 深拷贝
            self.goCartesian(waypoints)
            rospy.sleep(1)

            # # 旋转抓取角度
            # joint6 = 0 
            # if hori[i].theta >=0:
            #     joint6 = np.pi/2 - hori[i].theta
            # else:
            #     joint6 = -(np.pi/2 - abs(hori[i].theta))
            # print('theta:',hori[i].theta)
            # print('旋转角度为:',joint6)
            # self.moveJoint6(joint6)

            self.gripper.close_gripper()
            
            waypoints.clear()
            #*********放置*******
            # up
            wp.position.z +=0.2 
            waypoints.append(deepcopy(wp))
            if i == 0:
                # forward/back
                wp.position.y = self.position_dic['node0'][1]-0.098
                waypoints.append(deepcopy(wp))

                # left/right
                wp.position.x = self.position_dic['node0'][0]
                waypoints.append(deepcopy(wp))

                # down 
                wp.position.z = self.position_dic['node0'][2]+0.005 
                waypoints.append(deepcopy(wp))
                self.goCartesian(waypoints)
                rospy.sleep(1)
                self.gripper.open_gripper()
                waypoints.clear()
                # 记录放置的位置
                p_hori0 = self.getPosition()
                self.position_dic['hori0'][0] = p_hori0[0]
                self.position_dic['hori0'][1] = p_hori0[1]
                self.position_dic['hori0'][2] = p_hori0[2]

                rospy.sleep(1)
                self.goHome()
            if i == 1:
                # forward/back
                wp.position.y = self.position_dic['node0'][1]-0.098 
                waypoints.append(deepcopy(wp))

                # left/right
                wp.position.x = self.position_dic['node0'][0]+0.192
                waypoints.append(deepcopy(wp))

                # down 
                wp.position.z = self.position_dic['node0'][2]+0.005 
                waypoints.append(deepcopy(wp))
                self.goCartesian(waypoints)
                rospy.sleep(1)
                self.gripper.open_gripper()
                waypoints.clear()
                # 记录放置的位置
                p_hori1 = self.getPosition()
                self.position_dic['hori1'][0] = p_hori1[0]
                self.position_dic['hori1'][1] = p_hori1[1]
                self.position_dic['hori1'][2] = p_hori1[2]

                rospy.sleep(1)
                self.goHome()
            
            if i == 2:
                # forward/back
                wp.position.y = self.position_dic['node0'][1] 
                waypoints.append(deepcopy(wp))

                # left/right
                wp.position.x = self.position_dic['node0'][0]+0.098
                waypoints.append(deepcopy(wp))

                # down 
                wp.position.z = self.position_dic['node0'][2]+0.008
                waypoints.append(deepcopy(wp))
                self.goCartesian(waypoints)
                rospy.sleep(1)

                self.moveJoint6(1.570796327)
                self.gripper.open_gripper()
                waypoints.clear()
                # 记录放置的位置
                p_hori2 = self.getPosition()
                self.position_dic['hori2'][0] = p_hori2[0]
                self.position_dic['hori2'][1] = p_hori2[1]
                self.position_dic['hori2'][2] = p_hori2[2]

                rospy.sleep(1)
                self.goHome()
            if i == 3:
                # forward/back
                wp.position.y = self.position_dic['node0'][1]-0.192
                waypoints.append(deepcopy(wp))

                # left/right
                wp.position.x = self.position_dic['node0'][0]+0.098
                waypoints.append(deepcopy(wp))

                # down 
                wp.position.z = self.position_dic['node0'][2]+0.008
                waypoints.append(deepcopy(wp))
                self.goCartesian(waypoints)
                rospy.sleep(1)

                self.moveJoint6(1.570796327)
                self.gripper.open_gripper()
                waypoints.clear()
                # 记录放置的位置
                p_hori3 = self.getPosition()
                self.position_dic['hori3'][0] = p_hori3[0]
                self.position_dic['hori3'][1] = p_hori3[1]
                self.position_dic['hori3'][2] = p_hori3[2]
                rospy.sleep(1)
                self.goHome()
    # 第一层纵向筋
    def first_path_vertical(self,ver):
        (position,orientation) = self.getPose() # 基于base
        waypoints = []  # 初始化路径列表
        # # 计算平均深度值 规避深度值为0的点
        # depth_avg = 0.0
        # count = 0.0
        # for i in ver:
        #     if i.depth !=0:
        #         depth_avg +=i.depth
        #         count +=1
        # if count !=0:
        #     depth_avg /= count
        # else:
        #     print('4个深度均为0')
        #     return False
        # print('平均深度为：',depth_avg)
        # # 用平均值替代深度为0的点
        # for i in ver:
        #     if i.depth ==0:
        #         i.depth = depth_avg

        for i in range(0,len(ver)):
            if len(ver)!=4:
                print('识别数量有误：',len(ver))
                break
            print('抓取第{}个工件'.format(i+1))

            point = np.array([ver[i].center_x,ver[i].center_y,693],dtype=object)
            point_ = self.camera2base(point)
            # 写死z方向值
            point_[2] = 0.028
            print('要抓取的点:',point_)

            start_pose = Pose() 
            # 内容替换，保留格式
            start_pose.orientation.x = float(orientation[0])
            start_pose.orientation.y = float(orientation[1])
            start_pose.orientation.z = float(orientation[2])
            start_pose.orientation.w = float(orientation[3])
            # 抓取时补偿
            comp_z = 0.0
            comp_y = 0.0
            comp_x = 0.0
            if i==0:
                comp_y = -0.07
            if i ==1:
                comp_y = 0.07-0.008
            if i ==2:
                comp_y = -0.07
            if i==3:
                comp_y = 0.07-0.015

            # 初始化路径列表
            # waypoints.append(start_pose)
            
            # *******抓取******
            wp = deepcopy(start_pose)
            wp.position.x = float(point_[0])+0.008
            wp.position.y = float(point_[1])+comp_y # 存在误差需补偿
            wp.position.z = float(point_[2]) # 误差补偿
            waypoints.append(deepcopy(wp)) # 深拷贝
            self.goCartesian(waypoints)
            rospy.sleep(1)

            # # 旋转抓取角度
            # joint6 = 0 
            # if ver[i].theta >=0:
            #     joint6 = np.pi/2 - ver[i].theta
            # else:
            #     joint6 = -(np.pi/2 - abs(ver[i].theta))
            # print('theta:',ver[i].theta)
            # print('旋转角度为:',joint6)
            # self.moveJoint6(joint6)

            self.gripper.close_gripper()
            rospy.sleep(1)
            waypoints.clear()
            #*********放置*******
            # up
            wp.position.z +=0.3
            waypoints.append(deepcopy(wp))
            self.goCartesian(waypoints)
            waypoints.clear()
            rospy.sleep(1)

            if i == 0:
                # forward/back
                wp.position.y = self.position_dic['node0'][1]+0.002
                waypoints.append(deepcopy(wp))

                # left/right
                wp.position.x = self.position_dic['node0'][0]-0.001
                waypoints.append(deepcopy(wp))
                self.goCartesian(waypoints)
                waypoints.clear()
                rospy.sleep(1)
                
                # 切换姿态
                p = self.getPosition()
                rpy = [1.570796327,0,0]
                poes = p+rpy
                self.arm.set_pose_target(poes)
                self.arm.go()
                rospy.sleep(1)

                # down
                p1,ori1 = self.getPose()
                print("放置时的坐标点:",p1)
                # 内容替换
                start_pose.position.x = float(p1[0])
                start_pose.position.y = float(p1[1])
                start_pose.position.z = float(p1[2])
                start_pose.orientation.x = float(ori1[0])
                start_pose.orientation.y = float(ori1[1])
                start_pose.orientation.z = float(ori1[2])
                start_pose.orientation.w = float(ori1[3])
                wp1=deepcopy(start_pose)
                wp1.position.z =0.21
                waypoints.append(deepcopy(wp1))
                self.goCartesian(waypoints)
                waypoints.clear()
                # 记录放置时的位置
                p_ver0 = self.getPosition()
                self.position_dic['ver0'][0] = p_ver0[0]
                self.position_dic['ver0'][1] = p_ver0[1]
                self.position_dic['ver0'][2] = p_ver0[2]

                rospy.sleep(1)
                self.gripper.open_gripper()

                wp1.position.z +=0.15
                self.arm.set_pose_target(deepcopy(wp1))
                self.arm.go()
                rospy.sleep(1)
                self.goHome()

            if i == 1:
                # forward/back
                wp.position.y = self.position_dic['node1'][1]-0.007
                waypoints.append(deepcopy(wp))

                # left/right
                wp.position.x = self.position_dic['node1'][0]
                waypoints.append(deepcopy(wp))
                self.goCartesian(waypoints)
                waypoints.clear()
                rospy.sleep(1)
                
                # 切换姿态
                p = self.getPosition()
                rpy = [-1.570796327,0,0]
                poes = p+rpy
                self.arm.set_pose_target(poes)
                self.arm.go()
                rospy.sleep(1)

                # down
                p2,ori2 = self.getPose()
                print("放置时的坐标点:",p2)
                # 内容替换
                start_pose.position.x = float(p2[0])
                start_pose.position.y = float(p2[1])
                start_pose.position.z = float(p2[2])
                start_pose.orientation.x = float(ori2[0])
                start_pose.orientation.y = float(ori2[1])
                start_pose.orientation.z = float(ori2[2])
                start_pose.orientation.w = float(ori2[3])
                wp2=deepcopy(start_pose)
                wp2.position.z =0.21
                waypoints.append(deepcopy(wp2))
                self.goCartesian(waypoints)
                waypoints.clear()
                # 记录放置时的位置
                p_ver1 = self.getPosition()
                self.position_dic['ver1'][0] = p_ver1[0]
                self.position_dic['ver1'][1] = p_ver1[1]
                self.position_dic['ver1'][2] = p_ver1[2]

                rospy.sleep(1)
                self.gripper.open_gripper()
                wp2.position.z +=0.15
                self.arm.set_pose_target(deepcopy(wp2))
                self.arm.go()
                rospy.sleep(1)
                self.goHome()
            if i == 2:
                # forward/back
                wp.position.y = self.position_dic['node2'][1]+0.003
                waypoints.append(deepcopy(wp))

                # left/right
                wp.position.x = self.position_dic['node2'][0]
                waypoints.append(deepcopy(wp))
                self.goCartesian(waypoints)
                waypoints.clear()
                rospy.sleep(1)
                
                # 切换姿态
                p = self.getPosition()
                rpy = [1.570796327,0,0]
                poes = p+rpy
                self.arm.set_pose_target(poes)
                self.arm.go()
                rospy.sleep(1)

                # down
                p3,ori3 = self.getPose()
                print("放置时的坐标点:",p3)
                # 内容替换
                start_pose.position.x = float(p3[0])
                start_pose.position.y = float(p3[1])
                start_pose.position.z = float(p3[2])
                start_pose.orientation.x = float(ori3[0])
                start_pose.orientation.y = float(ori3[1])
                start_pose.orientation.z = float(ori3[2])
                start_pose.orientation.w = float(ori3[3])
                wp3=deepcopy(start_pose)
                wp3.position.z =0.21
                waypoints.append(deepcopy(wp3))
                self.goCartesian(waypoints)
                waypoints.clear()
                # 记录放置时的位置
                p_ver2 = self.getPosition()
                self.position_dic['ver2'][0] = p_ver2[0]
                self.position_dic['ver2'][1] = p_ver2[1]
                self.position_dic['ver2'][2] = p_ver2[2]

                rospy.sleep(1)
                self.gripper.open_gripper()
                wp3.position.z +=0.15
                self.arm.set_pose_target(deepcopy(wp3))
                self.arm.go()
                rospy.sleep(1)
                self.goHome()
            if i == 3:
                # forward/back
                wp.position.y = self.position_dic['node3'][1]-0.008-0.002
                waypoints.append(deepcopy(wp))

                # left/right
                wp.position.x = self.position_dic['node3'][0]
                waypoints.append(deepcopy(wp))
                self.goCartesian(waypoints)
                waypoints.clear()
                rospy.sleep(1)
                
                # 切换姿态
                p = self.getPosition()
                rpy = [-1.570796327,0,0]
                poes = p+rpy
                self.arm.set_pose_target(poes)
                self.arm.go()
                rospy.sleep(1)

                # down
                p4,ori4 = self.getPose()
                print("放置时的坐标点:",p3)
                # 内容替换
                start_pose.position.x = float(p4[0])
                start_pose.position.y = float(p4[1])
                start_pose.position.z = float(p4[2])
                start_pose.orientation.x = float(ori4[0])
                start_pose.orientation.y = float(ori4[1])
                start_pose.orientation.z = float(ori4[2])
                start_pose.orientation.w = float(ori4[3])
                wp4=deepcopy(start_pose)
                wp4.position.z =0.21
                waypoints.append(deepcopy(wp4))
                self.goCartesian(waypoints)
                waypoints.clear()
                # 记录放置时的位置
                p_ver3 = self.getPosition()
                self.position_dic['ver3'][0] = p_ver3[0]
                self.position_dic['ver3'][1] = p_ver3[1]
                self.position_dic['ver3'][2] = p_ver3[2]

                rospy.sleep(1)
                self.gripper.open_gripper()
                wp4.position.z +=0.15
                self.arm.set_pose_target(deepcopy(wp4))
                self.arm.go()
                rospy.sleep(1)
                self.goHome()
    
    # 执行工作空间路径
    def goPoint(self,objs):
        point = np.array([objs[0].center_x,objs[0].center_y,objs[0].depth],dtype=object)
        point_ = self.camera2base(point)
        print('point_:',point_)
        (position,orientation) = self.getPose() # 基于base
        start_pose = self.arm.get_current_pose().pose # 基于base_link
        # 内容替换
        start_pose.position.x = float(point_[0])
        start_pose.position.y = float(point_[1])
        start_pose.position.z = float(point_[2])
        start_pose.orientation.x = float(orientation[0])
        start_pose.orientation.y = float(orientation[1])
        start_pose.orientation.z = float(orientation[2])
        start_pose.orientation.w = float(orientation[3])
        print('target_point:',start_pose)
        # self.arm.set_pose_target(start_pose,end_effector_link=self.end_effector_link)
        # self.arm.go()
        # rospy.sleep(2)
        # self.arm.clear_pose_target(end_effector_link=self.end_effector_link)
    
    # 执行笛卡尔空间路径
    def goCartesian(self,waypoints):
        fraction = 0.0 # 路径规划覆盖率
        maxtrise = 100 # 最大规划次数
        attempts = 0  # 已经尝试规划的次数
        
        # 规划笛卡尔路径
        while fraction < 1.0 and attempts < maxtrise:
            (plan,fraction) = self.arm.compute_cartesian_path(waypoints,eef_step=0.01,jump_threshold=0.0,avoid_collisions=True)
            attempts +=1
            if attempts % 10 ==0:
                print('已经规划了{}',format(attempts))
        
        if fraction == 1.0:
            rospy.loginfo('规划成功')
            # print(plan.joint_trajectory.points[-4:])
            while plan.joint_trajectory.points[-1] == plan.joint_trajectory.points[-2]:
                    del plan.joint_trajectory.points[-1]
            self.arm.execute(plan)
        else:
            print('规划失败，路径覆盖率达到{}',format(fraction))
        pass
   
    # 四元数转旋转矩阵
    def q2tr(self,q):
        q_ = R.from_quat(q) # x，y，z,w
        transM= q_.as_matrix()
        return transM

    # 旋转矩阵转四元数
    def tr2q(self,tr):
        tr_ = R.from_matrix(tr) 
        q = tr_.as_quat() # x，y，z,w
        return q
    
    # 相机->工具->base的变换
    def camera2base(self,p):
        # 像素坐标->相机坐标系
        # p[2] *=self.cam_depth_scale
        p[2] *=0.001
        # print("像素坐标:",p) 
        pos_x = np.multiply(p[0] - self.cam_k[0][2],
                            p[2] / self.cam_k[0][0])
        pos_y = np.multiply(p[1] - self.cam_k[1][2],
                            p[2] / self.cam_k[1][1])

        # cam_k = np.linalg.inv(self.cam_k)
        # p_ = np.array([pos_x,pos_y,p[2],1.00],dtype=object)
        # p_.shape = (4,1)
        # print('点在相机坐标系下:',p_)

        # camera2tool = self.cam_pose
        # p_in_tool_ = np.dot(camera2tool, p_)
        # p_in_tool = p_in_tool_[0:3]
        # # print('点在工具坐标系下:',p_in_tool)

        # # tool0 ->base 的position trans
        # tool_position_,tool_rot = self.getPose()
        # # print("tool->base",tool_position_,tool_rot)
        # tool_rm = self.q2tr(tool_rot)
        # tool2rob = np.array(tool_rm)
        # tool_position = np.array(tool_position_)
        # tool_position = tool_position.reshape(3,1)
        # target_pose = np.dot(tool2rob[0:3, 0:3], p_in_tool) + tool_position
        # # print("点在机器人坐标系下：",target_pose)

        # ********camera_color_frame->base

        # camera ->base 的position trans
        p_ = np.array([pos_x,pos_y,p[2]],dtype=object)
        p_.shape = (3,1)
        # print('点在相机坐标系下:',p_)
        camera_position_,camera_rot = self.getPose_camera()
        # print("tool->base",tool_position_,tool_rot)
        camera_rm = self.q2tr(camera_rot)
        camera2rob = np.array(camera_rm)
        camera_position = np.array(camera_position_)
        camera_position = camera_position.reshape(3,1)
        target_pose = np.dot(camera2rob[0:3, 0:3], p_) + camera_position
        # print("点在机器人坐标系下：",target_pose)

        return target_pose
    
    # 获得工件的x图像坐标值
    def getPixel_x(self,elem):
        return elem.center_x
    
    # 获得工件的y图像坐标值
    def getPixel_y(self,elem):
        return elem.center_y
    
    # 旋转关节6角度
    def moveJoint6(self,rad):
        joint = self.arm.get_current_joint_values()
        joint[5] += rad
        self.arm.set_joint_value_target(joint)
        self.arm.go(wait=True)
        rospy.sleep(2)

    def moveJoint5(self,rad):
        joint = self.arm.get_current_joint_values()
        joint[4] += rad
        self.arm.set_joint_value_target(joint)
        self.arm.go(wait=True)
        rospy.sleep(2)
    # 节点抓放
    def node_grasp_pick(self,grasp_point,i):
        # 补偿
        if i == 0:
            com_x = 0.00
            com_y = 0.00
        if i == 1:
            com_x = 0.01
            com_y = 0.012
        if i == 2:
            com_x = 0.012
            com_y = 0.005
        if i == 3:
            com_x = 0.01
            com_y = 0.013

        grasp_point_base = self.camera2base(grasp_point)
        print("计算出的点in base:",grasp_point_base)
        start_pose = Pose()
        (position,orientation) = self.getPose() # 基于base
        start_pose.orientation.x = float(orientation[0])
        start_pose.orientation.y = float(orientation[1])
        start_pose.orientation.z = float(orientation[2])
        start_pose.orientation.w = float(orientation[3])
        start_pose.position.x = float(grasp_point_base[0])+0.00963+0.0024
        start_pose.position.y = float(grasp_point_base[1])+0.00741-0.008
        start_pose.position.z = 0.075
        print("补偿后的抓取点:",start_pose.position)
        self.arm.set_pose_target(start_pose)
        self.arm.go(wait=True)
        rospy.sleep(1)
        p,_ = self.getPose()
        print("到抓取点后返回的位置:",p)
        self.gripper.close_gripper()
        #*********放置*******
        wp = deepcopy(start_pose)
        waypoints = []
        # up
        up_z = 0
        if i in [0,1,2,3]:
            up_z = 0.2
        else:
            up_z = 0.4
        wp.position.z +=up_z 
        waypoints.append(deepcopy(wp))
        if i == 0:
            # left/right
            wp.position.x -=0.38
            waypoints.append(deepcopy(wp))

            # forward/back
            wp.position.y +=0.03
            waypoints.append(deepcopy(wp))

            # down 
            wp.position.z -=0.202
            waypoints.append(deepcopy(wp))
            self.goCartesian(waypoints)
            rospy.sleep(1)
            self.gripper.open_gripper()
            waypoints.clear()
            # 得到第一个node 放置时反馈的末端位置 基于base
            p_node0 = self.getPosition()
            self.position_dic['node0'][0] = p_node0[0]
            self.position_dic['node0'][1] = p_node0[1]
            self.position_dic['node0'][2] = p_node0[2]

            # up
            wp.position.z +=0.2 
            waypoints.append(deepcopy(wp))
            self.goCartesian(waypoints)
            rospy.sleep(1)

            # self.goHome()
            waypoints.clear()
            
        if i == 1:
            # forward/back
            wp.position.y = self.position_dic['node0'][1]-0.192+0.001 
            waypoints.append(deepcopy(wp))
            
            # left/right
            wp.position.x = self.position_dic['node0'][0]+0.001
            waypoints.append(deepcopy(wp))

            # down 
            wp.position.z -=0.204
            waypoints.append(deepcopy(wp))
            self.goCartesian(waypoints)
            rospy.sleep(1)
            self.gripper.open_gripper()
            waypoints.clear()
            p_node1 = self.getPosition()
            self.position_dic['node1'][0] = p_node1[0]
            self.position_dic['node1'][1] = p_node1[1]
            self.position_dic['node1'][2] = p_node1[2]
            # up
            wp.position.z +=0.2 
            waypoints.append(deepcopy(wp))
            self.goCartesian(waypoints)
            rospy.sleep(1)
            waypoints.clear()

        if i == 2:
            # forward/back
            wp.position.y = self.position_dic['node0'][1]+0.004
            waypoints.append(deepcopy(wp))
            
            # left/right
            wp.position.x = self.position_dic['node0'][0]+0.192 
            waypoints.append(deepcopy(wp))

            # down 
            wp.position.z -=0.2
            waypoints.append(deepcopy(wp))
            self.goCartesian(waypoints)
            rospy.sleep(1)
            self.gripper.open_gripper()
            waypoints.clear()
            p_node2 = self.getPosition()
            self.position_dic['node2'][0] = p_node2[0]
            self.position_dic['node2'][1] = p_node2[1]
            self.position_dic['node2'][2] = p_node2[2]

            # up
            wp.position.z +=0.2 
            waypoints.append(deepcopy(wp))
            self.goCartesian(waypoints)
            rospy.sleep(1)
            waypoints.clear()
        if i == 3:
            # forward/back
            wp.position.y = self.position_dic['node0'][1]-0.192+0.004
            waypoints.append(deepcopy(wp))

            # left/right
            wp.position.x = self.position_dic['node0'][0]+0.192
            waypoints.append(deepcopy(wp))

            # down 
            wp.position.z -=0.2
            waypoints.append(deepcopy(wp))
            self.goCartesian(waypoints)
            rospy.sleep(1)
            self.gripper.open_gripper()
            waypoints.clear()
            p_node3 = self.getPosition()
            self.position_dic['node3'][0] = p_node3[0]
            self.position_dic['node3'][1] = p_node3[1]
            self.position_dic['node3'][2] = p_node3[2]

            # up
            wp.position.z +=0.2 
            waypoints.append(deepcopy(wp))
            self.goCartesian(waypoints)
            rospy.sleep(1)
            waypoints.clear()
        if i == 4:
            # forward/back
            # wp.position.y = self.position_dic['node0'][1]-0.005
            wp.position.y = self.position_dic['node0'][1]
            waypoints.append(deepcopy(wp))

            # left/right
            # wp.position.x = self.position_dic['ver0'][0]+0.005
            wp.position.x = self.position_dic['node0'][0]
            waypoints.append(deepcopy(wp))

            # down 
            wp.position.z =0.272
            waypoints.append(deepcopy(wp))
            self.goCartesian(waypoints)
            self.second_1node = self.getPosition()
            print("第二层第一个node放置坐标,",self.second_1node)
            rospy.sleep(1)
            waypoints.clear()
            self.gripper.open_gripper()

            # up
            wp.position.z +=0.2 
            waypoints.append(deepcopy(wp))
            self.goCartesian(waypoints)
            rospy.sleep(1)
            waypoints.clear()
        if i == 5:
            # forward/back
            # wp.position.y = self.position_dic['ver1'][1]+0.006
            wp.position.y = self.position_dic['node1'][1]
            waypoints.append(deepcopy(wp))

            # left/right
            # wp.position.x = self.position_dic['ver1'][0]+0.004
            wp.position.x = self.position_dic['node1'][0]
            waypoints.append(deepcopy(wp))

            # down 
            wp.position.z =0.272
            waypoints.append(deepcopy(wp))
            self.goCartesian(waypoints)
            print("当前坐标：",self.getPosition())
            rospy.sleep(1)
            waypoints.clear()
            self.gripper.open_gripper()

            # up
            wp.position.z +=0.2 
            waypoints.append(deepcopy(wp))
            self.goCartesian(waypoints)
            rospy.sleep(1)
            waypoints.clear()
        if i == 6:
            # forward/back
            wp.position.y = self.position_dic['node2'][1]
            waypoints.append(deepcopy(wp))

            # left/right
            wp.position.x = self.position_dic['node2'][0]
            waypoints.append(deepcopy(wp))

            # down 
            wp.position.z =0.272
            waypoints.append(deepcopy(wp))
            self.goCartesian(waypoints)
            rospy.sleep(1)
            print("当前坐标：",self.getPosition())
            waypoints.clear()
            self.gripper.open_gripper()

            # up
            wp.position.z +=0.2 
            waypoints.append(deepcopy(wp))
            self.goCartesian(waypoints)
            rospy.sleep(1)
            waypoints.clear()
        if i == 7:
            # forward/back
            # wp.position.y = self.position_dic['ver3'][1]+0.002
            wp.position.y = self.position_dic['node3'][1]
            waypoints.append(deepcopy(wp))

            # left/right
            # wp.position.x = self.position_dic['ver3'][0]+0.004
            wp.position.x = self.position_dic['node3'][0]
            waypoints.append(deepcopy(wp))

            # down 
            wp.position.z =0.272
            waypoints.append(deepcopy(wp))
            self.goCartesian(waypoints)
            print("当前坐标：",self.getPosition())
            rospy.sleep(1)
            waypoints.clear()
            self.gripper.open_gripper()

            # up
            wp.position.z +=0.2 
            waypoints.append(deepcopy(wp))
            self.goCartesian(waypoints)
            rospy.sleep(1)
            waypoints.clear()
    
    # 第二层横向筋
    def second_path_horizontal(self,hori):
        (position,orientation) = self.getPose() # 基于base
        waypoints = []  # 初始化路径列表
        # 计算平均深度值 规避深度值为0的点
        depth_avg = 0.0
        count = 0.0
        if len(hori)!=8:
            print('识别数量有误：',len(hori))
            return
        for i in hori:
            if i.depth !=0:
                depth_avg +=i.depth
                count +=1
        if count !=0:
            depth_avg /= count
        else:
            print('4个深度均为0')
            return False
        print('平均深度为：',depth_avg)
        # 用平均值替代深度为0的点
        for i in hori:
            if i.depth ==0:
                i.depth = depth_avg

        for i in range(4,8):
            print('抓取第{}个工件'.format(i+1))

            point = np.array([hori[i].center_x,hori[i].center_y,693],dtype=object)
            point_ = self.camera2base(point)
            # 写死z方向值
            point_[2] = 0.034
            print('要抓取的点:',point_)

            start_pose = Pose() 
            # 内容替换，保留格式
            start_pose.position.x = float(position[0])
            start_pose.position.y = float(position[1])
            start_pose.position.z = float(position[2])
            start_pose.orientation.x = float(orientation[0])
            start_pose.orientation.y = float(orientation[1])
            start_pose.orientation.z = float(orientation[2])
            start_pose.orientation.w = float(orientation[3])
            # 抓取时补偿
            comp_z = 0.0
            comp_y = 0.0
            comp_x = 0.0
            if i==4:
                comp_y = 0.00
            if i ==5:
                comp_y = -0.002
            if i ==6:
                comp_y = -0.001
            if i==7:
                comp_y = -0.002


            # 初始化路径列表
            # waypoints.append(start_pose)
            
            # *******抓取******
            wp = deepcopy(start_pose)
            wp.position.y = float(point_[1])+comp_y # 存在误差需补偿
            waypoints.append(deepcopy(wp))

            wp.position.x = float(point_[0])
            waypoints.append(deepcopy(wp))

            wp.position.z = float(point_[2]) 
            waypoints.append(deepcopy(wp)) # 深拷贝

            self.goCartesian(waypoints)
            rospy.sleep(1)

            # # 旋转抓取角度
            # joint6 = 0 
            # if hori[i].theta >=0:
            #     joint6 = np.pi/2 - hori[i].theta
            # else:
            #     joint6 = -(np.pi/2 - abs(hori[i].theta))
            # print('theta:',hori[i].theta)
            # print('旋转角度为:',joint6)
            # self.moveJoint6(joint6)

            self.gripper.close_gripper()
            
            waypoints.clear()
            #*********放置*******
            # up
            wp.position.z +=0.4 
            waypoints.append(deepcopy(wp))
            if i == 4:
                # forward/back
                wp.position.y = self.second_1node[1]-0.098
                waypoints.append(deepcopy(wp))

                # left/right
                wp.position.x = self.second_1node[0]
                waypoints.append(deepcopy(wp))

                # down 
                wp.position.z = self.second_1node[2]+0.005 
                waypoints.append(deepcopy(wp))
                self.goCartesian(waypoints)
                rospy.sleep(1)
                self.gripper.open_gripper()
                waypoints.clear()
                rospy.sleep(1)
                self.goHome()
            if i == 5:
                # forward/back
                wp.position.y = self.second_1node[1]-0.098 
                waypoints.append(deepcopy(wp))

                # left/right
                wp.position.x = self.second_1node[0]+0.192
                waypoints.append(deepcopy(wp))

                # down 
                wp.position.z = self.second_1node[2]+0.003 
                waypoints.append(deepcopy(wp))
                self.goCartesian(waypoints)
                rospy.sleep(1)
                self.gripper.open_gripper()
                waypoints.clear()
                rospy.sleep(1)
                self.goHome()
            
            if i == 6:
                # forward/back
                wp.position.y = self.second_1node[1] 
                waypoints.append(deepcopy(wp))

                # left/right
                wp.position.x = self.second_1node[0]+0.098
                waypoints.append(deepcopy(wp))

                # down 
                wp.position.z = self.second_1node[2]+0.008
                waypoints.append(deepcopy(wp))
                self.goCartesian(waypoints)
                rospy.sleep(2)

                self.moveJoint6(1.570796327)
                self.gripper.open_gripper()
                waypoints.clear()
                # 记录放置的位置
                p_hori2 = self.getPosition()
                self.position_dic['hori2'][0] = p_hori2[0]
                self.position_dic['hori2'][1] = p_hori2[1]
                self.position_dic['hori2'][2] = p_hori2[2]

                rospy.sleep(1)
                self.goHome()
            if i == 7:
                # forward/back
                wp.position.y = self.second_1node[1]-0.192
                waypoints.append(deepcopy(wp))

                # left/right
                wp.position.x = self.second_1node[0]+0.098
                waypoints.append(deepcopy(wp))

                # down 
                wp.position.z = self.second_1node[2]+0.008
                waypoints.append(deepcopy(wp))
                self.goCartesian(waypoints)
                rospy.sleep(2)

                self.moveJoint6(1.570796327)
                self.gripper.open_gripper()
                waypoints.clear()
                rospy.sleep(1)
                self.goHome()
    # 回调函数
    def callBack(self,obj):
        if self.grasping.flag == False:
            self.grasping.flag = True
            self.grasping_pub.publish(self.grasping) 
            print('执行抓取ing')
        #  定义类型  
        #  工件归属
            for i in obj.objects:
                if i.cls == 0:
                    self.horizontals.objects.append(deepcopy(i))
                elif i.cls == 1:
                    self.nodes.objects.append(deepcopy(i))
                elif i.cls == 2:
                    self.verticals.objects.append(deepcopy(i))
            # 按x从左至右进行排序
            self.nodes.objects.sort(key=self.getPixel_x)
            self.horizontals.objects.sort(key=self.getPixel_x)
            self.verticals.objects.sort(key=self.getPixel_x)
            
            # 第一层节点
            # home状态下对每个节点进行第一次的base坐标计算
            for i in range(0,len(self.nodes.objects)):
                if self.nodes.objects[i].depth == 0:
                    self.nodes.objects[i].depth = 653
                    print("深度为0,补偿为653")
                p=[self.nodes.objects[i].center_x,self.nodes.objects[i].center_y,653]
                p_base = self.camera2base(p)
                self.node_position_first.append(p_base)
            waypoints=[]
            # 对节点进行二次拍照
            print("对节点进行二次拍照并抓取")
            for i in range(0,len(self.nodes.objects)):
                if i==4:
                    break
                print("节点的数量:",len(self.nodes.objects))
                (position,orientation) = self.getPose() # 基于base
                start_pose = Pose()
                start_pose.orientation.x = float(orientation[0])
                start_pose.orientation.y = float(orientation[1])
                start_pose.orientation.z = float(orientation[2])
                start_pose.orientation.w = float(orientation[3])
                start_pose.position.x = float(self.node_position_first[i][0])
                start_pose.position.y = float(self.node_position_first[i][1])-0.096
                start_pose.position.z = 0.605
                print("start_pose:",start_pose.position)
                # waypoints.append(deepcopy(start_pose))
                # self.goCartesian(waypoints)
                self.arm.set_pose_target(start_pose)
                self.arm.go(wait=True)
                rospy.sleep(2)
                waypoints.clear()
                response = self.client.call(320,240) # 中心点
                rospy.sleep(1)
                print("第",i+1,"个:",response)
                grasp_point = [response.x_r,response.y_r,653]
                self.node_grasp_pick(deepcopy(grasp_point),i)
            
            rospy.sleep(1)
            self.goHome() 
            rospy.sleep(1)
            print("各工件的放置点:\n",self.position_dic)
            # 第一层横向筋
            print("横向筋的个数:",len(self.horizontals.objects)) 
            self.first_path_horizontal(self.horizontals.objects)

            # 第一层纵向筋
            print("纵向筋的个数:",len(self.verticals.objects)) 
            self.first_path_vertical(self.verticals.objects) 
            print("第一层各工件的放置点:\n",self.position_dic)

            # 第二层节点
            for i in range(4,len(self.nodes.objects)):
                if i ==8:
                    break
                (position,orientation) = self.getPose() # 基于base
                start_pose = Pose()
                start_pose.orientation.x = float(orientation[0])
                start_pose.orientation.y = float(orientation[1])
                start_pose.orientation.z = float(orientation[2])
                start_pose.orientation.w = float(orientation[3])
                start_pose.position.x = float(self.node_position_first[i][0])
                start_pose.position.y = float(self.node_position_first[i][1])-0.096
                start_pose.position.z = 0.605
                print("start_pose:",start_pose.position)
                # waypoints.append(deepcopy(start_pose))
                # self.goCartesian(waypoints)
                self.arm.set_pose_target(start_pose)
                self.arm.go(wait=True)
                rospy.sleep(2)
                waypoints.clear()
                response = self.client.call(320,240) # 中心点
                rospy.sleep(1)
                print("第",i+1,"个:",response)
                grasp_point = [response.x_r,response.y_r,653]
                self.node_grasp_pick(deepcopy(grasp_point),i)
            rospy.sleep(1)
            self.goHome() 
            rospy.sleep(1)

            key = input("是否进行下一流程?(1:是; 其它:否)")
            if key!="1":
                rospy.signal_shutdown('测试完成')
            else:
                print("继续进行下一流程")
            # 第二层横杆
            self.second_path_horizontal(self.horizontals.objects)


   
if __name__ == "__main__":
    test = moveitCartesianDemo()
    test.goHome()
    rospy.sleep(1)

    rospy.spin()

