import rospy
from moveit_commander import MoveGroupCommander
from copy import deepcopy
from geometry_msgs.msg import Pose
import tf
class test_calibration():
    def __init__(self) -> None:
        rospy.init_node('calibretion')
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
        
        # tf监听
        self.listener = tf.TransformListener()
    
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

    # ready位
    def goReady(self):   
        self.arm.set_named_target('ready')
        self.arm.go(wait=True)
        rospy.sleep(1)

    # 回home位
    def goHome(self):   
        self.arm.set_named_target('home')
        self.arm.go(wait=True)
        rospy.sleep(1)
    
    # 回0位
    def goZero(self):   
        self.arm.set_named_target('zero')
        self.arm.go(wait=True)
        rospy.sleep(1)
    
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
    

if __name__ == "__main__":
    ur10 = test_calibration()
    # # 回零位
    # ur10.goZero()
    # rospy.sleep(1)

    # ready
    # ur10.goReady()
    rospy.sleep(5)
    # # 得到当前位姿
    position,ori=ur10.getPose()
    # # 定义初始点
    star_pose = Pose()
    star_pose.position.x = float(position[0])
    star_pose.position.y = float(position[1])
    star_pose.position.z = float(position[2])
    star_pose.orientation.x = float(ori[0])
    star_pose.orientation.y = float(ori[1])
    star_pose.orientation.z = float(ori[2])
    star_pose.orientation.w = float(ori[3])

    waypoints = []
    # waypoints.append(star_pose)

    wp = deepcopy(star_pose)  # -160 -800 400  -90  0  -180
    # 斜面矩形轨迹
    # x方向-30cm
    wp.position.x -=0.3 # -460 -800 400  -90  0  -180
    waypoints.append(deepcopy(wp))
    ur10.goCartesian(waypoints)
    rospy.sleep(8)


    # 斜向下
    wp.position.y += 0.3 # -460 -500 200  -90  0  -180
    wp.position.z -=0.2
    waypoints.append(deepcopy(wp))
    ur10.goCartesian(waypoints)
    waypoints.clear()
    rospy.sleep(8)

    
    # x方向+30cm
    wp.position.x +=0.3 # -160 -500 200  -90  0  -180 
    waypoints.append(deepcopy(wp))
    ur10.goCartesian(waypoints)
    waypoints.clear()
    rospy.sleep(8)


    # 中心点
    wp.position.x -=0.15  # -310 -650 210  -90  0  -180
    wp.position.y -=0.15
    wp.position.z +=0.10
    waypoints.append(deepcopy(wp))
    ur10.goCartesian(waypoints)
    waypoints.clear()
    rospy.sleep(8)


    # 终点
    wp.position.x +=0.15
    wp.position.y -= 0.15
    wp.position.z +=0.10
    waypoints.append(deepcopy(wp))
    ur10.goCartesian(waypoints)
    waypoints.clear()
    rospy.sleep(8)

    # ur10.goCartesian(waypoints)
    print("测试结束！")
