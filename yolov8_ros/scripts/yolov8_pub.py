#!/home/lj/anaconda3/envs/pytorch/bin/python
from ultralytics import YOLO 
import rospy
from yolov8_ros.msg import objectArray,workpieces,grasp_flag
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv_bridge
from copy import deepcopy
from PIL import Image as image_p
import math
import torch
from yolov8_ros.srv import center,centerResponse,ibvs,ibvsResponse
from std_msgs.msg import Float64MultiArray,Float32MultiArray

class getBox():
    def __init__(self):
        # 定义参数
        weight_path = '/home/lj/project/ur_ws/src/best_steel2.pt'
        image_topic = '/camera/color/image_raw'
        depth_topic = '/camera/aligned_depth_to_color/image_raw'
        pub_topic = 'workpieces'
        conf = 0.25 # 置信度阈值
        self.visualize = True
        self.cv_bridge = cv_bridge.CvBridge()
        self.model = YOLO(weight_path)
        self.model_getCenter = YOLO(weight_path)
        self.model_getCenter.fuse()
        self.model.fuse()   # 提升模型推理速度
        self.model.conf = conf
        self.depth_img = Image()    # 深度图对象
        self.color_image = Image() # 彩图对象
        self.grasping = grasp_flag()
        self.grasping.flag = False
        self.grasping.cls = 0
        self.grasping.conf = 0.8
        
        # 创建订阅与发布对象
        self.sub_flag = rospy.Subscriber('grasping',grasp_flag,self.graspingCB,queue_size=1)
        rospy.sleep(1)
        self.des_vector = rospy.Subscriber("desir_vector",Float32MultiArray,self.desir_vectorCB,queue_size=5)
        self.sub_obs_piexl = rospy.Subscriber("obs_piexl",Float32MultiArray,self.obs_piexl,queue_size=5)
        self.sub_depth = rospy.Subscriber(depth_topic,Image,self.getDepth,queue_size=1)
        self.image_sub = rospy.Subscriber(image_topic,Image,self.callBack,queue_size=1,buff_size=2**24)
        self.msg_pub = rospy.Publisher(pub_topic,objectArray,queue_size=1)
        self.ibvs_pub = rospy.Publisher('ibvs_data',Float64MultiArray,queue_size=5) # 发布ibvs数据

        self.ibvs_data = Float64MultiArray() # 发布ibvs数据
        self.detection = workpieces()
        self.workpieceArray = objectArray()
        self.flag = False # 识别到工件标志位
        # 创建服务端 用于二次拍照获得精确坐标
        self.server = rospy.Service("center",center,self.getCenter)
        self.server_ibvs = rospy.Service("ibvs_srv",ibvs,self.ibvs_srv)
        self.ibvs_response = ibvsResponse()
        self.ibvs_strat_flag = False
        self.grasping_flag = False
        self.canvas = np.ones((1280, 1500, 3), dtype=np.uint8) * 255
        self.piexl_list = Float32MultiArray()
        self.des_vector = Float32MultiArray()
        print("服务器center,ibvs已启动")

        # ibvs特征期望点
        # self.points = [(255, 48), (638, 48), (255, 478), (638, 478)] # 装配使用
        # self.points = [(263, 164), (406, 164), (263, 308), (406, 308)] #cbf期望特征点
        self.points = [(158, 54), (529, 54), (158, 422), (529, 422)]
        # 初始化ibvs四个角点的历史轨迹存储
        self.corner_histories = [[] for _ in range(4)]  # [左上, 右上, 右下, 左下]
        self.max_history_length = 3000  # 最大轨迹长度
    def ibvs_srv(self,requset):
        if requset.ibvs_request: # true 表示开始ibvs计算
            self.ibvs_strat_flag = True
            self.ibvs_response = True
            print("ibvs请求开启!")
        else:
            self.ibvs_strat_flag = False
            self.ibvs_response = False
            print("ibvs请求关闭!")
        return self.ibvs_response

    # 服务端回调函数,用于二次拍照 
    def getCenter(self,requset):
        response = centerResponse()
        results = deepcopy(self.model_getCenter.predict(self.color_image,show=False,conf=self.grasping.conf,classes=[self.grasping.cls],verbose=False))
        self.rectify(results)
        # plot_image = results[0].plot()
        # num = 0
        boxes = results[0].boxes.cpu().numpy()
        image_center = np.array([320, 240])
        min_distance = float('inf')
        closest_bbox = None
        for box in boxes:  # 遍历每一个检测框，提取距离中心最近的检测框坐标
            # 当检测的框为重复时，跳过此框
            # if num in index:
            #     num +=1
            #     continue
            # 获取边界框坐标
            if int(box.cls) != requset.cls:
                print("类别不对***********************",type(requset.cls),type(int(box.cls)),int(box.cls))
                continue
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            bbox_center = np.array([(x1 + x2) // 2, (y1 + y2) // 2])
            distance = np.linalg.norm(bbox_center - image_center)
            if distance < min_distance:
                min_distance = distance
                closest_bbox = box

        if closest_bbox == None:
            response.flag = False
            response.x_r = requset.x
            response.y_r = requset.y
            return response
        x1, y1, x2, y2 = map(int,closest_bbox.xyxy[0])
        # 金属+++++++++++
        # print(x1, y1, x2, y2)
        response.x_r = (x1+x2)/2
        response.y_r = (y1+y2)/2
        # response.flag = True
        # cv2.imshow("plot_image",plot_image)
        # cv2.waitKey(0)
        # return response
        # 金属+++++++++++
        # 提取ROI (Region of Interest)，即感兴趣区域
        roi = self.color_image[y1:y2, x1:x2]
        # 计算该区域内的轮廓
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=6, tileGridSize=(8,8))
        enhanced = clahe.apply(gray)
        # blurred = cv2.GaussianBlur(enhanced, (5,5), 0)
        # thresh = cv2.adaptiveThreshold(blurred, 255, 
        #                   cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        #                   cv2.THRESH_BINARY, 11, 2)
        edges = cv2.Canny(enhanced, 200, 600)

        # 形态学闭合操作连接边缘
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 计算该区域内的轮廓
        # gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        # _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        # contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # 找到最大的轮廓
            c = max(contours, key=cv2.contourArea)
            # 计算最小外接矩形
            rect = cv2.minAreaRect(c)
            box_pts = cv2.boxPoints(rect)
            box_pts = box_pts.astype("int")
            center = rect[0]
            center = (int(center[0]+x1), int(center[1]+y1)) 
            # 绘制最小外接矩形
            for pt in box_pts:
                # 将每个顶点坐标转换为基于整张图像的绝对坐标
                pt[0] += x1
                pt[1] += y1
            # response.x_r = center[0]
            # response.y_r = center[1]
            print(rect[2])
            if rect[1][0]<rect[1][1]:
                response.angle = rect[2] - 90
            else:
                response.angle = rect[2]
            response.flag = True
        else:
            response.flag = False
        # cv2.drawContours(plot_image, [box_pts], 0, (0, 255, 0), 2)
        # cv2.imshow('Result_srv', plot_image)
        # cv2.waitKey(0)
        return response

    # 抓取标志回调函数 
    def graspingCB(self,msg):
        self.grasping.flag = msg.flag
        self.grasping.conf = msg.conf
        self.grasping.cls = msg.cls
        print("已接收抓取标志请求")
    # 显示避障像素点
    def show_obs(self,img):
        self.canvas[:] = 255
        h, w = img.shape[:2]
        x_offset = (1500 - w) // 2
        y_offset = (1280 - h) // 2
        # print(y_offset)
        self.canvas[y_offset:y_offset+h, x_offset:x_offset+w] = img
        if len(self.piexl_list.data) < 1:
            return
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 1
        # 画出tool0与obs
        danger_color = (0, 100, 255)  # 橙红色（比纯红更警示，常用于危险标识）
        margin_color = (0, 180, 255)  # 更黄一点，温和警示
        v = np.array([self.des_vector.data[0],self.des_vector.data[1]])
        v = v / np.linalg.norm(v)
        arrow_len = 50
        end_point = (int(367 +x_offset+ v[0] * arrow_len), int( 719+y_offset+ v[1] * arrow_len))
        # 画箭头
        cv2.arrowedLine(self.canvas, (367 + x_offset,719 + y_offset), end_point, color=(0, 255, 0), thickness=2, tipLength=0.2)
        cv2.circle(self.canvas, (367 + x_offset,719 + y_offset), 3, (0, 0, 0), -1) # tool0
        cv2.putText(self.canvas, "tool0", (367 + x_offset - 20, 719 + y_offset - 20), font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)

        for i in range(1,len(self.piexl_list.data),5):
            center = (int(self.piexl_list.data[i])+x_offset,int(self.piexl_list.data[i+1]+y_offset))
            radius = int(self.piexl_list.data[i+2])
            radius_margin = int(self.piexl_list.data[i+3])
            cv2.circle(self.canvas, center, radius, danger_color, 2)
            if self.piexl_list.data[0] < self.piexl_list.data[i+4]: # 当tool0低于obs high时画裕度圆
                cv2.circle(self.canvas, center, radius_margin, margin_color, 2)
            cv2.circle(self.canvas, center, 3, danger_color, -1)  # thickness = -1 表示实心
            cv2.putText(self.canvas, f"OBS{i//3}", (center[0] - 20, center[1] - radius - 10), font, font_scale, danger_color, thickness, cv2.LINE_AA)
        pass 

    # 识别及处理数据显示结果的回调函数 
    def callBack(self,msg):
        self.color_image = self.cv_bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
        results = deepcopy(self.model.predict(self.color_image,show=False,conf=self.grasping.conf,classes=[self.grasping.cls],verbose=False))
        self.rectify(results)
        img = results[0].plot()
        if self.grasping.flag == True:
            self.comput_xy_angle(results)
            print("222")
            self.workpieceArray.objects.clear()
            self.grasping.flag = False
        cv2.namedWindow("results", cv2.WINDOW_NORMAL)
        cv2.imshow("results", img)
        cv2.waitKey(2)
    
    # ibvs_cbf
    def  comput_ibvs_cbf(self,img):
        results_node=deepcopy(self.model.predict(self.color_image,show=False,conf=0.8,classes=[0],verbose=False))
        boxes = results_node[0].boxes.cpu().numpy()
        self.ibvs_data.data.clear()
        for box in boxes:  # 遍历每一个检测框
            if boxes.cls.size != 1:
                self.ibvs_data.data = [-1]
                self.ibvs_pub.publish(self.ibvs_data)
                return
            # 获取边界框坐标
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            # # 提取ROI (Region of Interest)，即感兴趣区域
            # roi = self.color_image[y1:y2, x1:x2]
            # # 计算该区域内的轮廓
            # gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            # clahe = cv2.createCLAHE(clipLimit=6, tileGridSize=(8,8))
            # enhanced = clahe.apply(gray)
            # edges = cv2.Canny(enhanced, 140, 420)
            # # 形态学闭合操作连接边缘
            # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
            # closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
            # contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # if len(contours) > 0:
            #     # 找到最大的轮廓
            #     c = max(contours, key=cv2.contourArea)
            #     # 计算最小外接矩形
            #     rect = cv2.minAreaRect(c)
            #     box_pts = cv2.boxPoints(rect)
            #     box_pts = box_pts.astype("int")
            #     # 绘制最小外接矩形
            #     i = 0
            #     for pt in box_pts:
            #         # 将每个顶点坐标转换为基于整张图像的绝对坐标
            #         pt[0] += x1
            #         pt[1] += y1
            #         # print(pt," ",i)
            #         # i += 1
            #     cv2.drawContours(img, [box_pts], 0, (0, 255, 0), 2)
            # else:
            #     self.ibvs_data.data = [-1]
            #     self.ibvs_pub.publish(self.ibvs_data)
            #     return
            # 用均值替代深度为0的点
            tepm_x2 = x2
            temp_y2 = y2
            points = (self.depth_img[(temp_y2-1),(tepm_x2-50):tepm_x2])
            points_ = points[points > 0]
            depth_depth = np.mean(points_) if len(points_) > 0 else 0

            self.ibvs_data.data.append(x1)
            self.ibvs_data.data.append(y1)

            self.ibvs_data.data.append(x2)
            self.ibvs_data.data.append(y1)

            self.ibvs_data.data.append(x1)
            self.ibvs_data.data.append(y2)

            self.ibvs_data.data.append(x2)
            self.ibvs_data.data.append(y2)

            # self.ibvs_data.data.append(box_pts[0][0])
            # self.ibvs_data.data.append(box_pts[0][1])

            # self.ibvs_data.data.append(box_pts[1][0])
            # self.ibvs_data.data.append(box_pts[1][1])

            # self.ibvs_data.data.append(box_pts[2][0])
            # self.ibvs_data.data.append(box_pts[2][1])

            # self.ibvs_data.data.append(box_pts[3][0])
            # self.ibvs_data.data.append(box_pts[3][1])

            # self.ibvs_data.data.append(new_points[0]*0.001)
            self.ibvs_data.data.append(depth_depth*0.001)
            self.ibvs_data.data.append(depth_depth*0.001)
            self.ibvs_data.data.append(depth_depth*0.001)
            self.ibvs_data.data.append(depth_depth*0.001)
            self.ibvs_pub.publish(self.ibvs_data)
            # print(self.ibvs_data.data[:8])

    # 用于发送ibvs数据
    def  comput_ibvs(self):
        results_node=deepcopy(self.model.predict(self.color_image,show=False,conf=0.8,classes=[0],verbose=False))
        img = results_node[0].plot()
        # filtered_boxes = [box for box in results_node[0].boxes if box.cls == 0]
        # results_node[0].boxes = filtered_boxes
        boxes = results_node[0].boxes.cpu().numpy()
        self.ibvs_data.data.clear()
        if boxes.cls.size == 1:
            x1, y1, x2, y2 = map(int, boxes.xyxy[0])
            # 用均值替代深度为0的点
            # print(x1, y1, x2, y2)
            tepm_x1 = x1
            temp_y1 = y1
            points = (self.depth_img[(temp_y1-1),(tepm_x1-50):tepm_x1])
            points_ = points[points > 0]
            depth_depth = np.mean(points_) if len(points_) > 0 else 0
            # points = [self.depth_img[y1,x1-1],self.depth_img[y2,x1-1],self.depth_img[y1,x2-1],self.depth_img[y2,x2-1]]
            # non_zero_values = [p for p in points if p != 0]
            # avg_value = sum(non_zero_values) / len(non_zero_values)
            # new_points = [p if p != 0 else avg_value for p in points]

            self.ibvs_data.data.append(x1)
            self.ibvs_data.data.append(y1)

            self.ibvs_data.data.append(x2)
            self.ibvs_data.data.append(y1)

            self.ibvs_data.data.append(x1)
            self.ibvs_data.data.append(y2)

            self.ibvs_data.data.append(x2)
            self.ibvs_data.data.append(y2)

            # self.ibvs_data.data.append(new_points[0]*0.001)
            self.ibvs_data.data.append(depth_depth*0.001)
            self.ibvs_data.data.append(depth_depth*0.001)
            self.ibvs_data.data.append(depth_depth*0.001)
            self.ibvs_data.data.append(depth_depth*0.001)
        else:
            self.ibvs_data.data = [-1]
        self.ibvs_pub.publish(self.ibvs_data)
        # cv2.imshow('ibvs_trajectory', img)
        # cv2.waitKey(2)

    # 计算并绘制每个识别框的中心点与角度
    def comput_xy_angle(self,results):
        plot_image = results[0].plot()
        # self.workpieceArray.objects.clear()
        num = 0
        boxes = results[0].boxes.cpu().numpy()
        for box in boxes:  # 遍历每一个检测框
            # print(box)
            # 当检测的框为重复时，跳过此框
            # if num in index:
            #     num +=1
            #     continue

            # 获取边界框坐标
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            # print(x1, y1, x2, y2)
            # print("depth1",self.depth_img[y1,x1])
            # print("depth2",self.depth_img[y2,x2])
            # 金属件的中心点计算
            # print("x1, y1, x2, y2",x1, y1, x2, y2)
            center_steel = ((x2-x1)/2,(y2-y1)/2)

            # 提取ROI (Region of Interest)，即感兴趣区域
            roi = self.color_image[y1:y2, x1:x2]

            # 计算该区域内的轮廓
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=6, tileGridSize=(8,8))
            enhanced = clahe.apply(gray)
            # blurred = cv2.GaussianBlur(enhanced, (5,5), 0)
            # thresh = cv2.adaptiveThreshold(blurred, 255, 
            #                   cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            #                   cv2.THRESH_BINARY, 11, 2)
            edges = cv2.Canny(enhanced, 200, 600)

            # 形态学闭合操作连接边缘
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
            closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # cv2.imshow("gray", gray)
            # cv2.moveWindow("gray", 50, 50)
            # cv2.imshow("enhanced", enhanced)
            # cv2.moveWindow("enhanced", 400, 50)
            # cv2.imshow("Edges", edges)
            # cv2.moveWindow("Edges", 50, 300)
            # cv2.imshow("closed", closed)
            # cv2.moveWindow("closed", 400, 300)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            # gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            # _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
            # contours, _ = cv2.findContours(adaptive_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                # 找到最大的轮廓
                c = max(contours, key=cv2.contourArea)
                # 计算最小外接矩形
                rect = cv2.minAreaRect(c)
                box_pts = cv2.boxPoints(rect)
                box_pts = box_pts.astype("int")
                center = rect[0]
                # center = (int(center[0]+x1), int(center[1]+y1)) 
                center = (int(center_steel[0]+x1), int(center_steel[1]+y1)) # 金属
                # 绘制最小外接矩形
                for pt in box_pts:
                    # 将每个顶点坐标转换为基于整张图像的绝对坐标
                    pt[0] += x1
                    pt[1] += y1
                # cv2.drawContours(plot_image, [box_pts], 0, (0, 255, 0), 2)
                # 可选：计算角度等信息
                # if rect[1][0]<rect[1][1]:
                #     angle = rect[2] - 90
                # else:
                angle = rect[2]
                # print(angle)
                text = f"Angle: {angle:.2f} deg"
                text_position = (x1 + 10, y1 + 30)
                font = cv2.FONT_HERSHEY_SIMPLEX
                # cv2.putText(plot_image, text, text_position, font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                # cv2.circle(plot_image, center, 1, (255, 0, 0), -1)  # 绘制中心点
            
            # 赋值该识别框内工件的信息 
            self.detection.center_x = center[0]
            self.detection.center_y = center[1]
            self.detection.cls = int(box.cls)
            self.detection.conf = float(box.conf)
            self.detection.depth = self.depth_img[center[1],center[0]] # 索引方式（row，col）
            self.detection.theta = angle
            if self.detection.cls == 0 :
                # self.detection.name = "horizontal"
                self.detection.name = "node" # 金属
            if self.detection.cls == 1 :
                # self.detection.name = "node"
                self.detection.name = "vertical" # 金属
                # print(self.detection.center_x,self.detection.center_y)
            if self.detection.cls == 2 :
                # self.detection.name = "vertical"
                self.detection.name = "horizontal" # 金属
            # 添加至工件容器内 
            self.workpieceArray.objects.append(deepcopy(self.detection)) # 务必深拷贝
            num +=1
        self.msg_pub.publish(self.workpieceArray)
        # cv2.imshow('Result', plot_image)
        # cv2.waitKey(0)

    # 获取yolo结果中的数据
    def creat_detect_array(self,results):
        if len(results[0].boxes.cls) == 0:
            print('未检测到工件')
            return
        # print(results[0].boxes.xywh)
        # 修正并删除重复项
        index = self.rectify(results)
        bounding_box = results[0].boxes.xywh # 这帧图像上所有框的信息，类型为张量
        classes = results[0].boxes.cls
        confidence_score = results[0].boxes.conf
        # 获取各工件数据
        num = 0
        for bbox, cls, conf in zip(bounding_box, classes, confidence_score):  #将列表中对应元素进行压缩 并解包 
            # 规避掉index中重复下标的box
            if num in index:
                # print("num:",num)
                # print(bbox)
                num +=1
                continue
            self.detection.center_x = float(bbox[0])
            self.detection.center_y = float(bbox[1])
            self.detection.width = float(bbox[2])
            self.detection.height = float(bbox[3])
            self.detection.cls = int(cls)
            self.detection.conf = float(conf)
            self.workpieceArray.objects.append(deepcopy(self.detection)) # 务必深拷贝
            num +=1
        # print(self.workpieceArray.objects)
        # 最初偏移的中心点 绿色
        for i in range(0,len(self.workpieceArray.objects)):
            if self.workpieceArray.objects[i].cls ==1:   
                cv2.circle(results[0].orig_img,(round(self.workpieceArray.objects[i].center_x),round(self.workpieceArray.objects[i].center_y)),1,(0,255,0),1)
        
        # 获取对应深度
        for i in range(0,len(self.workpieceArray.objects)):
            self.workpieceArray.objects[i].depth = self.depth_img[round(self.workpieceArray.objects[i].center_y),round(self.workpieceArray.objects[i].center_x)] # 索引方式（row，col）
        # print('{}的深度值为：{},像素为：{}'.format(self.workpieceArray.objects[0].cls,self.workpieceArray.objects[0].depth,(round(self.workpieceArray.objects[0].center_x),round(self.workpieceArray.objects[0].center_y))))
        
        # 赋值对应工件名称及角度
        for i in range(0,len(self.workpieceArray.objects)):
            if self.workpieceArray.objects[i].cls == 0:
                self.workpieceArray.objects[i].name ="horizontal"
                self.workpieceArray.objects[i].theta = self.getBoxAngle(results,self.workpieceArray.objects[i])
                # print(self.workpieceArray.objects[i])
            if self.workpieceArray.objects[i].cls == 1:
                self.workpieceArray.objects[i].name ="node"
                # self.workpieceArray.objects[i].theta = self.getNodeAngle(results,self.workpieceArray.objects[i])
                self.workpieceArray.objects[i].center_x = self.centerComp_x(self.workpieceArray.objects[i])
                self.workpieceArray.objects[i].center_y = self.centerComp_y(self.workpieceArray.objects[i])
                # print(f'已偏差圆心{self.workpieceArray.objects[i].center_x,self.workpieceArray.objects[i].center_y}')
                # self.getNodeCenter(results,self.workpieceArray.objects[i])
                # print('节点像素中心点：',(self.workpieceArray.objects[i].center_x,self.workpieceArray.objects[i].center_y),' 深度值：',self.workpieceArray.objects[i].depth)
            if self.workpieceArray.objects[i].cls == 2:
                self.workpieceArray.objects[i].name ="vertical"
                self.workpieceArray.objects[i].theta = self.getBoxAngle(results,self.workpieceArray.objects[i])
                # print(self.workpieceArray.objects[i])
        # 发布消息
        self.msg_pub.publish(self.workpieceArray)
   
    # 获取深度图
    def getDepth(self,msg):
        self.depth_img = self.cv_bridge.imgmsg_to_cv2(msg)
        # print('已获取深度图,类型为:{}',type(self.depth_img))
 
    # 显示图像    
    def creat_result_image(self,results):
        plot_image = results[0].plot()
        for i in self.workpieceArray.objects:
                cv2.circle(plot_image,(round(i.center_x),round(i.center_y)),1,(255,0,0),1)
        # for i in self.workpieceArray.objects:
        #     self.getNodeCenter(results,i,plot_image)
        cv2.imshow('yolov8',plot_image)
        cv2.waitKey(0)

    # 修正识别错误的竖杆
    def rectify(self,results):
        # 如果该工件为竖杆但是框的斜边长度小于173，则识别错误，纠正为横杆
        for i in range(0,len(results[0].boxes.cls)):
            if results[0].boxes.cls[i] == 1:
                   if  200 <= math.sqrt(results[0].boxes.xywh[i][2]**2 + results[0].boxes.xywh[i][3]**2):
                        # print(results[0].boxes.xywh[i][0])
                        # print("该竖杆识别无误")
                        pass
                   else:
                        results[0].boxes.cls[i] = 2
                        # print(results[0].boxes.xywh[i][0])
                        # print("该竖杆识别有误！")
        # 找出重复的中心点对，并取前一个中心点的下标
        # index = []
        # for i in range(0,len(results[0].boxes.xywh)):
        #     for j in range(0,len(results[0].boxes.xywh)):
        #         if i==j:
        #             continue
        #         if j in index:
        #             continue
        #         temp_xywh = results[0].boxes.xywh[i] -results[0].boxes.xywh[j]
        #         if abs(temp_xywh[0])<2 and abs(temp_xywh[1])<2:
        #             print("i-j:",i,j)
        #             index.append(i)
        # print("index:",index)
        # return index

    # 绘制ibvs轨迹
    def ibvs_trajectory(self,img):
        # trajectory
        if len(self.ibvs_data.data) < 2:
            return
        # 计算四个角点坐标（左上、右上、右下、左下）
        current_corners = [
            (self.ibvs_data.data[0], self.ibvs_data.data[1]),  # 左上 (点1)
            (self.ibvs_data.data[2], self.ibvs_data.data[3]),  # 右上 (点2)
            (self.ibvs_data.data[4], self.ibvs_data.data[5]),  # 左下 (点3)
            (self.ibvs_data.data[6], self.ibvs_data.data[7])   # 右下 (点4)
        ]

        # 更新历史轨迹
        for i in range(4):
            self.corner_histories[i].append(current_corners[i])
            # 保持历史轨迹长度不超过设定值
            if len(self.corner_histories[i]) > self.max_history_length:
                self.corner_histories[i].pop(0)

        # 绘制轨迹和标记
        if any(len(h) > 0 for h in self.corner_histories):
            for corner_idx in range(4):
                history = self.corner_histories[corner_idx]
                
                # 绘制轨迹线
                for j in range(1, len(history)):
                    prev = tuple(map(int, history[j-1]))
                    curr = tuple(map(int, history[j]))
                    cv2.line(img, prev, curr, (0, 255, 0), 1)
        pass

    # obs像素回调
    def obs_piexl(self,piexl_list_):
        self.piexl_list = piexl_list_

    # 期望向量回调
    def desir_vectorCB(self,desir_vector_):
        self.des_vector = desir_vector_

def main():
    rospy.init_node("test_pub")
    # print('延时5s...')
    # rospy.sleep(5)
    getbox=getBox()
    # while((not rospy.is_shutdown())) # rospy.is_shutdown() 返回 False，表示 ROS 仍在运行
    rospy.spin()  
if __name__ == "__main__":
    main()