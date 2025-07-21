#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
class MoveItCartesianDemo:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('test', anonymous=True)

        listener = tf.TransformListener()
        rospy.sleep(1)

        print("tool->world\n")
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
        #     rospy.logerr("Failed to get transform between %s and %s", 'base_link', source_fr

        # 控制机械臂先回到初始化位置
        # arm.set_named_target('ready')
        # arm.go()
        # rospy.sleep(2)
                                               
        target_frame = 'camera_color_frame'
        source_frame = 'camera_color_optical_frame'
        print("-------------\n")
        try:
            listener.waitForTransform(target_frame,source_frame,rospy.Time(0),rospy.Duration(1))
            (trans1, rot1) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            print("Translation: ", trans1)
            print("Rotation: ", rot1)
        except:
            rospy.logerr("Failed to get transform between %s and %s", target_frame, source_frame)
        # 获取当前位姿数据最为机械臂运动的起始位姿
        rospy.is_shutdown()

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass