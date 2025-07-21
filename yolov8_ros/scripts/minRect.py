import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
import math

def show_image(image, title):
    """显示图像"""
    plt.imshow(image, cmap='gray')
    plt.title(title)
    plt.axis('off')
    plt.show()

def calculate_centroid_of_column(image):
    """计算每列255像素的重心坐标"""
    centroids = []
    for x in range(image.shape[1]):
        y_indices = np.where(image[:, x] == 255)[0]
        if len(y_indices) > 0:
            centroid_y = np.mean(y_indices)
            centroids.append((x, centroid_y))
    return np.array(centroids)

def calculate_centroid_of_row(image):
    """计算每行255像素的重心坐标"""
    centroids = []
    for y in range(image.shape[0]):
        x_indices = np.where(image[y, :] == 255)[0]
        if len(x_indices) > 0:
            centroid_x = np.mean(x_indices)
            centroids.append((centroid_x, y))
    return np.array(centroids)

def fit_line_least_squares(points):
    """通过最小二乘法拟合直线，返回直线的斜率和截距"""
    x = points[:, 0]
    y = points[:, 1]
    slope, intercept, _, _, _ = stats.linregress(x, y)
    return slope, intercept

def find_intersection(slope1, intercept1, slope2, intercept2):
    """计算两条直线的交点"""
    x = (intercept2 - intercept1) / (slope1 - slope2)
    y = slope1 * x + intercept1
    return x, y


def find_rectangle_boundaries(image, horizontal_slope, horizontal_intercept, vertical_slope, vertical_intercept, centroid):
    """通过二分法寻找上下左右边界"""
    # 计算旋转中心的交点
    rotation_center = find_intersection(horizontal_slope, horizontal_intercept, vertical_slope, vertical_intercept)
    
    # 上下边界
    y_upper = int(rotation_center[1] + 50)
    y_lower = int(rotation_center[1] - 50)
    
    # 通过主轴的斜率计算垂直线的斜率
    vertical_slope_perpendicular = -1 / horizontal_slope
    b_vertical_perpendicular = rotation_center[1] - vertical_slope_perpendicular * rotation_center[0]
    
    # 计算左右边界
    x_left = int((y_lower - b_vertical_perpendicular) / vertical_slope_perpendicular)
    x_right = int((y_upper - b_vertical_perpendicular) / vertical_slope_perpendicular)
    
    return rotation_center, x_left, x_right, y_lower, y_upper


def move_hori(horizontal_slope,horizontal_intercept,img_binary):
    # 白色区域点集合
    # pixels = np.where(img_binary == 255)
    # white_pixels = set(zip(pixels[0], pixels[1]))

    x_vals = np.linspace(0, img_binary.shape[1] - 1, 100) # 线性生成X坐标
    # 计算主轴的首尾坐标
    y_init = horizontal_slope * x_vals[0] + horizontal_intercept
    y_final = horizontal_slope * (img_binary.shape[1] - 1) + horizontal_intercept
    # 初始平移量
    shift_up = y_init
    shift_down = img_binary.shape[0] - y_final
    # 用于存储直线上的点
    flag_down = True
    flag_up = True
    # 最近边界的截距
    bound = [0,0]
    # 下边界
    while flag_down:
        # down 计算平移后的截距
        b_shift_down_init = (y_final+shift_down) - horizontal_slope * x_vals[-1]

        # 绘制平移后的主轴
        y_vals = horizontal_slope * x_vals + b_shift_down_init
        for i in range(len(x_vals)):
            x = int(x_vals[i])
            y = int(y_vals[i])
            if 0 <= x < binary.shape[1] and 0 <= y < binary.shape[0] and img_binary[y, x] == 255:
                flag_down = False
                bound[0] = b_shift_down_init
            # if 0 <= x < binary.shape[1] and 0 <= y < binary.shape[0]:
            #     img_binary[y, x] = 128  # 设置为灰色-------------------
        shift_down -= 15 
        # print(y_init,y_final)
        # return display_image
    while flag_up:
        # up计算平移后的截距
        b_shift_up_init = (y_init-shift_up) - horizontal_slope * x_vals[0]

        # 绘制平移后的主轴
        y_vals = horizontal_slope * x_vals + b_shift_up_init
        for i in range(len(x_vals)):
            x = int(x_vals[i])
            y = int(y_vals[i])
            if 0 <= x < binary.shape[1] and 0 <= y < binary.shape[0] and img_binary[y, x] == 255:
                flag_up = False
                bound[1] = b_shift_up_init
            # if 0 <= x < binary.shape[1] and 0 <= y < binary.shape[0]:
            #     img_binary[y, x] = 128  # 设置为灰色-------------------
        shift_up -= 15 
    # print(bound)
    return bound

def move_ver(horizontal_slope,rotation_center,img_binary):
    # 法线
    Normal_slope = -1/horizontal_slope
    Normal_intercept = rotation_center[1] - rotation_center[0]*Normal_slope

    y_vals = np.linspace(0, img_binary.shape[0] - 1, 100) # 线性生成y坐标
    # x_vals = (y_vals - b_shift_left_init) / Normal_slope
    # 计算主轴的首尾坐标
    x_init = (y_vals[0] - Normal_intercept)/Normal_slope
    x_final = ((img_binary.shape[0] - 1) - Normal_intercept)/Normal_slope
    # 初始平移量
    shift_right = img_binary.shape[1] - x_init
    shift_left = x_final
    # 用于存储直线上的点
    flag_right = True
    flag_left = True
    # 最近边界的截距
    bound = [0,0]
    # 右边界
    while flag_right:
        # down 计算平移后的截距
        b_shift_right_init = (img_binary.shape[0] - 1) - (x_init+shift_right)*Normal_slope
        # 绘制平移后的主轴
        x_vals = (y_vals - b_shift_right_init) / Normal_slope
        for i in range(len(x_vals)):
            x = int(x_vals[i])
            y = int(y_vals[i])
            if 0 <= x < binary.shape[1] and 0 <= y < binary.shape[0] and img_binary[y, x] == 255:
                flag_right = False
                bound[0] = b_shift_right_init
            # if 0 <= x < binary.shape[1] and 0 <= y < binary.shape[0]:
            #     img_binary[y, x] = 128  # 设置为灰色-------------------
        shift_right -=2 

    while flag_left:
        # down 计算平移后的截距
        b_shift_left_init = y_vals[0] - (x_final - shift_left)*Normal_slope
        # 绘制平移后的主轴
        x_vals = (y_vals - b_shift_left_init) / Normal_slope
        for i in range(len(x_vals)):
            x = int(x_vals[i])
            y = int(y_vals[i])
            if 0 <= x < binary.shape[1] and 0 <= y < binary.shape[0] and img_binary[y, x] == 255:
                flag_left = False
                bound[1] = b_shift_left_init
            # if 0 <= x < binary.shape[1] and 0 <= y < binary.shape[0]:
            #     img_binary[y, x] = 128  # 设置为灰色-------------------
        shift_left -=5
    return bound

def rotate_point(y, x, angle, center):
    # 计算旋转矩阵
    angle_rad = math.radians(angle)  # 角度转换为弧度
    cos_angle = math.cos(angle_rad)
    sin_angle = math.sin(angle_rad)
    
    # 旋转点坐标
    x_new = cos_angle * (x - center[0]) + sin_angle * (y - center[1]) + center[0]
    y_new = -sin_angle * (x - center[0]) + cos_angle * (y - center[1]) + center[1]
    return int(x_new), int(y_new)

# 读取图像
image = cv2.imread('/home/lj/图片/minRect_Color.png')  # 请替换为你实际的图像路径
# image = cv2.imread('/home/lj/图片/ndoe_minRect_Color.png')  # 请替换为你实际的图像路径

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
_, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

# Step 1: 计算每列和每行的重心
column_centroids = calculate_centroid_of_column(binary)
row_centroids = calculate_centroid_of_row(binary)

# Step 2: 使用最小二乘法拟合水平主轴和垂直主轴
horizontal_slope, horizontal_intercept = fit_line_least_squares(column_centroids)
vertical_slope, vertical_intercept = fit_line_least_squares(row_centroids)

angle = math.atan(vertical_slope) - math.atan(horizontal_slope)
degree = math.degrees(angle)
print("degree:",degree)

# Step 3: 计算旋转中心
rotation_center, x_left, x_right, y_lower, y_upper = find_rectangle_boundaries(binary, horizontal_slope, horizontal_intercept, vertical_slope, vertical_intercept, column_centroids.mean(axis=0))
center = (int(rotation_center[0]), int(rotation_center[1]))

# Step 4: 绘制主轴线、旋转中心和矩形边界
output_image = binary.copy()

# 绘制水平主轴
x_vals = np.linspace(0, binary.shape[1] - 1, 100)  # 修正为在范围内
y_vals = horizontal_slope * x_vals + horizontal_intercept
for i in range(len(x_vals)):
    x = int(x_vals[i])
    y = int(y_vals[i])
    if 0 <= x < binary.shape[1] and 0 <= y < binary.shape[0]:
        output_image[y, x] = 128  # 设置为灰色-------------------

# 平移主轴寻找矩形
intercept_hori = move_hori(horizontal_slope,horizontal_intercept,output_image)
intercept_ver = move_ver(horizontal_slope,rotation_center,output_image)

# 绘制初始矩形
for i in range(2):
    if i == 0 :
        y_vals = horizontal_slope * x_vals + intercept_hori[0]
    if i == 1 :
        y_vals = horizontal_slope * x_vals + intercept_hori[1]
    for i in range(len(x_vals)):
        x = int(x_vals[i])
        y = int(y_vals[i])
        if 0 <= x < binary.shape[1] and 0 <= y < binary.shape[0]:
            output_image[y, x] = 254  # 设置为灰色-------------------
        # for k in range(4):
        #     x_new,y_new = rotate_point(x,y,1+k,center)
        #     if 0 <= x_new < binary.shape[1] and 0 <= y_new < binary.shape[0]:
        #         output_image[x_new, y_new] = 128  # 设置为灰色-------------------
        for k in range(int(degree)):
            if k % 1 == 0:
                x_new,y_new = rotate_point(x,y,1+k,center)
                if 0 <= x_new < binary.shape[1] and 0 <= y_new < binary.shape[0]:
                    output_image[x_new, y_new] = 254  # 设置为灰色-------------------

y_vals = np.linspace(0, binary.shape[0] - 1, 100) # 线性生成y坐标
for i in range(4):
    if i == 0 :
        x_vals = (y_vals - intercept_ver[0]) / (-1/horizontal_slope)
    if i == 1 :
        x_vals = (y_vals - intercept_ver[1]) / (-1/horizontal_slope)
    for i in range(len(x_vals)):
        x = int(x_vals[i])
        y = int(y_vals[i])
        if 0 <= x < binary.shape[1] and 0 <= y < binary.shape[0]:
            output_image[y, x] = 254  # 设置为灰色-------------------
        # for k in range(4):
        #     x_new,y_new = rotate_point(x,y,1+k,center)
        #     if 0 <= x_new < binary.shape[1] and 0 <= y_new < binary.shape[0]:
        #         output_image[x_new, y_new] = 128  # 设置为灰色-------------------
        for k in range(int(degree)):
            if k % 1 == 0:
                x_new,y_new = rotate_point(x,y,1+k,center)
                if 0 <= x_new < binary.shape[1] and 0 <= y_new < binary.shape[0]:
                    output_image[x_new, y_new] = 254  # 设置为灰色-------------------



# 绘制垂直主轴
y_vals = np.linspace(0, binary.shape[0] - 1, 100)  # 修正为在范围内
x_vals = (y_vals - vertical_intercept) / vertical_slope
for i in range(len(x_vals)):
    x = int(x_vals[i])
    y = int(y_vals[i])
    if 0 <= x < binary.shape[1] and 0 <= y < binary.shape[0]:
        output_image[y, x] = 128  # 设置为灰色



# 绘制旋转中心
# output_image[int(rotation_center[1]), int(rotation_center[0])] = 255
cv2.circle(output_image,center, 2, 0, -1)
# 绘制矩形边界
# cv2.line(output_image, (x_left, y_lower), (x_left, y_upper), 255, 2)
# cv2.line(output_image, (x_left, y_lower), (x_right, y_lower), 255, 2)
# cv2.line(output_image, (x_right, y_lower), (x_right, y_upper), 255, 2)
# cv2.line(output_image, (x_left, y_upper), (x_right, y_upper), 255, 2)

# 显示最终的图像
show_image(output_image, "Bounding Box with Axes")