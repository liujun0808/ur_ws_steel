import rosbag
import rospy
from geometry_msgs.msg import WrenchStamped
def write():
    bag = rosbag.Bag('/home/lj/project/ur_ws/rosbag_force',)
    pass

def callBack(msg):
    print(msg.wrench.force.z)

def main():
    rospy.init_node("tets")
    sub_workpieces = rospy.Subscriber("transformed_world",WrenchStamped,callBack,queue_size=1)
    rospy.spin()
    pass
    
if __name__ == "__main__":
    main()
