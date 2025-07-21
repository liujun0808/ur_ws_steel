import cv2
import numpy as np

# 读取图像并转换为灰度图

# 读取彩色图像
image_color = cv2.imread('/home/lj/图片/node_Color.png')
# image_color = cv2.imread('/home/lj/图片/ver01_Color.png')
# image_color = cv2.imread('/home/lj/图片/hori_Color.png')

# 将彩色图像转换为灰度图
image_gray = cv2.cvtColor(image_color, cv2.COLOR_BGR2GRAY)

_, binary = cv2.threshold(image_gray, 127, 255, cv2.THRESH_BINARY)

# 查找轮廓
contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 初始化最大轮廓和其面积
max_contour = None
max_area = 0


# 遍历每个轮廓
for contour in contours:
    # 计算轮廓的面积
    area = cv2.contourArea(contour)
    
    # 更新最大轮廓
    if area > max_area:
        max_area = area
        max_contour = contour

# 如果找到了最大轮廓
if max_contour is not None:
    # 计算最大轮廓的最小外接矩形
    rect = cv2.minAreaRect(max_contour)
    box = cv2.boxPoints(rect)
    box = np.int0(box) #转换为整数
    center = rect[0]
    center = (int(center[0]), int(center[1])) 

    # 获取矩形的角度
    angle = rect[2]
    if rect[1][0] < rect[1][1]:
        angle = 90 - angle

    print(f"Max Contour Angle: {angle} degrees")

    # 在彩色图像上绘制最小外接矩形
    # 写文字
    font = cv2.FONT_HERSHEY_SIMPLEX
    text = f"Angle: {angle:.2f} deg"
    text_position = (center[0] , center[1] + 70)
    cv2.putText(image_color, text, text_position, font, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

    cv2.drawContours(image_color, [box], 0, (0, 0, 255), 2)
    cv2.circle(image_color, center, 1, (0, 255, 0), -1)  # 绘制中心点
# 显示结果
cv2.imshow('Contours with Min Area Rectangles', image_color)
cv2.waitKey(0)
cv2.destroyAllWindows()