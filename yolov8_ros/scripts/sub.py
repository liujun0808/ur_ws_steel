#! /usr/bin/env python
import rospy
from yolov8_ros.msg import objectArray,workpieces
from copy import deepcopy

class dataProcess():
    def __init__(self):
        self.sub_workpieces = rospy.Subscriber("workpieces",objectArray,self.callBack,queue_size=1)

        self.grasp_flag = False

        # 创建工件分类器
        self.nodes = objectArray()
        self.verticals =  objectArray()
        self.horizontals = objectArray()   
    # 第一层横向筋
    def first_path_horizontal(self,horizontals):
        print('horizontal:',horizontals)
        pass
    
    # 第一层纵向筋
    def first_path_vertical(self,verticals):
        print('vertical:',verticals)
        pass
    def getPixel_x(self,elem):
        return elem.center_x
    
    def callBack(self,obj):
            if self.grasp_flag == False:
                print('执行抓取')
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
                # 第一次层节点
                # self.first_path_node(self.nodes.objects)  
                # 第一层横向筋
                self.first_path_horizontal(self.horizontals.objects)
                # 第一层纵向筋
                self.first_path_vertical(self.verticals.objects)
                
            rospy.signal_shutdown('测试完成')   


if __name__ == "__main__":
    rospy.init_node("test_sub")
    data_process = dataProcess()
    rospy.spin()
