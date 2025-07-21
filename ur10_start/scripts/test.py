#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy
import tf

class MoveItCartesianDemo:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('test', anonymous=True)

        listener = tf.TransformListener()
        rospy.sleep(1)

        # print("tool->world\n")
        # try:
        #     listener.waitForTransform('world',source_frame,rospy.Time(0),rospy.Duration(1))
        #     (trans, rot) = listener.lookupTransform('world', source_frame, rospy.Time(0))
        #     print("Translation: ", trans)
        #     print("Rotation: ", rot)
        # except:
        #     rospy.logerr("Failed to get transform between %s and %s", 'world', source_frame)

        # print("tool->base_link\n")
        # try:
        #     listener.waitForTransform('base_link',source_frame,rospy.Time(0),rospy.Duration(1))
        #     (trans, rot) = listener.lookupTransform('base_link', source_frame, rospy.Time(0))
        #     print("Translation: ", trans)
        #     print("Rotation: ", rot)
        # except:
        #     rospy.logerr("Failed to get transform between %s and %s", 'base_link', source_frame)
        
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('manipulator')
        
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame('base')
    
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
        
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.1)
        arm.set_max_velocity_scaling_factor(0.1)
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

        # 控制机械臂先回到初始化位置
        # arm.set_named_target('ready')
        # arm.go()
        # rospy.sleep(3)
                                               
        target_frame = 'base'
        source_frame = 'tool0'
        print("tool->base\n")
        try:
            listener.waitForTransform(target_frame,source_frame,rospy.Time(0),rospy.Duration(1))
            (trans1, rot1) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            print("Translation: ", trans1)
            print("Rotation: ", rot1)
        except:
            rospy.logerr("Failed to get transform between %s and %s", target_frame, source_frame)
        # 获取当前位姿数据最为机械臂运动的起始位姿
        start_pose = arm.get_current_pose(end_effector_link).pose
        # start_pose.orientation = rot1
        # start_pose.position = trans1
        start_pose.orientation[0] = rot1[0]
        start_pose.orientation[1] = rot1[1]
        start_pose.orientation[2] = rot1[2]
        start_pose.orientation[3] = rot1[3]

        start_pose.position.x = trans1[0]
        start_pose.position.y = trans1[1]
        start_pose.position.z = trans1[2]

        print (end_effector_link,'\n')
        print ('当前的位姿\n',start_pose,'\n')

        # 初始化路点列表
        waypoints = []
                
        # 将初始位姿加入路点列表
        waypoints.append(start_pose)
            
        # 设置路点数据，并加入路点列表
        wpose = deepcopy(start_pose)
        wpose.position.x -= 0.2
        wpose.position.y -= 0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.x += 0.05
        wpose.position.y += 0.15
        wpose.position.z -= 0.15       
        waypoints.append(deepcopy(wpose))

        fraction = 0.0   #路径规划覆盖率
        maxtries = 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
 
        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.01,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划
            
            # 尝试次数累加
            attempts += 1
            
            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                     
        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        rospy.sleep(1)

        # 控制机械臂先回到初始化位置
        # arm.set_named_target('zero')
        # arm.go()
        # rospy.sleep(1)
        # point = PointStamped()
        # point.header.frame_id = 'world'
        # point.point.x = arm.get_current_pose(end_effector_link).pose.position.x
        # point.point.y = arm.get_current_pose(end_effector_link).pose.position.y
        # point.point.z = arm.get_current_pose(end_effector_link).pose.position.z 

        # transformed_point = listener.transformPoint(target_frame,point)
        # print ('运动后的tool->base_link',transformed_point.point,'\n')
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass