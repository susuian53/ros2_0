from time import sleep
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import sys

class ImageHandler:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_received = False
        self.current_image = None

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image_received = True


def QRcode():
    """
    识别二维码

    returns:
        int: 
            A-1返回1
            A-2返回2
            B-1返回3
            B-2返回4
    """
    # 创建图像处理程序并设置ROS订阅
    image_handler = ImageHandler()
    
    # 初始化ROS节点
    rclpy.init(args=None)
    node = rclpy.create_node('image_listener')
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )
    subscription = node.create_subscription(
        Image,
        '/rgb_camera/image_raw',
        image_handler.image_callback,
        qos_profile
    )
    
    # 等待接收第一张图像
    while not image_handler.image_received:
        rclpy.spin_once(node)
    image = image_handler.current_image
    
    # 截取上半部分
    height, width = image.shape[:2]
    upper_half = image[0:height//2, 0:width]

    gray = cv2.cvtColor(upper_half, cv2.COLOR_BGR2GRAY)
    equal = cv2.equalizeHist(gray)
    thresh = cv2.threshold(equal, 1, 255, cv2.THRESH_BINARY)[1]
    
    decoded = decode(thresh)
    result = 0
    if decoded:
        qr_data = decoded[0].data.decode('utf-8')
        node.get_logger().info(f"识别到二维码: {qr_data}")
        if qr_data == "A-1":
            result = 1
            node.get_logger().info("识别结果: A-1 -> 返回 1")
        elif qr_data == "A-2":
            result = 2
            node.get_logger().info("识别结果: A-2 -> 返回 2")
        elif qr_data == "B-1":
            result = 3
            node.get_logger().info("识别结果: B-1 -> 返回 3")
        elif qr_data == "B-2":
            result = 4
            node.get_logger().info("识别结果: B-2 -> 返回 4")
        else:
            node.get_logger().warn(f"未知二维码: {qr_data}")
    else:
        node.get_logger().warn("未能识别到二维码")
    

    # 清理资源
    node.destroy_subscription(subscription)
    rclpy.shutdown()
    return result

def get_ready():
    """
    通过交互判断是否完成动作

    returns:
        int: A区装货完成返回1,B区卸货完成返回2,B区装货完成返回3,A区卸货完成返回4
    """
    sleep(4)
    for i in range(1,6):
        print(f"装货中:{i}")
        sleep(1)
    return 1

def arrow():
    # 创建图像处理程序并设置ROS订阅
    image_handler = ImageHandler()
    
    # 初始化ROS节点
    rclpy.init(args=None)
    node = rclpy.create_node('image_listener')
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )
    subscription = node.create_subscription(
        Image,
        '/rgb_camera/image_raw',
        image_handler.image_callback,
        qos_profile
    )
    
    # 等待接收第一张图像
    while not image_handler.image_received:
        rclpy.spin_once(node)
    image = image_handler.current_image
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # 定义绿色HSV范围
    lower_green = np.array([35, 50, 50])
    upper_green = np.array([85, 255, 255])
    
    # 创建绿色掩膜
    mask = cv2.inRange(hsv, lower_green, upper_green)

    # 开运算去除噪声，闭运算连接区域
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # 查找轮廓
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    
    # 过滤小轮廓
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 100]
    
    if not contours:
        return 1# 默认
    
    # 取最大轮廓（假设箭头是最大的绿色区域）
    largest_contour = max(contours, key=cv2.contourArea)
    # 计算最小外接矩形
    rect = cv2.minAreaRect(largest_contour)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    
    # 计算轮廓的凸包和凸缺陷
    hull = cv2.convexHull(largest_contour, returnPoints=False)
    defects = cv2.convexityDefects(largest_contour, hull)
    if defects is None or len(defects) < 3:
        # 如果没有明显凸缺陷，使用外接矩形方向
        angle = rect[2]
        if angle < -45:
            angle += 90
        
        if -90 <= angle < 90:
            direction = "right"
        else:
            direction = "left"
    else:
        # 使用凸缺陷方法确定箭头尖端
        farthest_point = None
        max_dist = 0
        
        for i in range(defects.shape[0]):
            f, d = defects[i, 0][2:4]
            far = tuple(largest_contour[f][0])
            
            if d > max_dist:
                max_dist = d
                farthest_point = far
        
        if farthest_point:
            # 计算质心
            M = cv2.moments(largest_contour)
            cx = int(M['m10']/M['m00'])
            
            # 计算方向向量
            dx = farthest_point[0] - cx
            direction = "right" if dx > 0 else "left"
        else:
            direction = "unknown"
    node.get_logger().info(f"识别结果: 方向为{direction}")
    # 清理资源
    node.destroy_subscription(subscription)
    rclpy.shutdown()
    return 1 if direction == "left" else 2

def yellow_light():
    """
    进行语音播报(5,4,3,2,1)

    returns:
        int: 播报完成返回1
    """
    camera_height = 0.3  # 相机高度
    light_height = 0.6   # 黄灯高度
    # 创建图像处理程序并设置ROS订阅
    image_handler = ImageHandler()
    
    # 初始化ROS节点
    rclpy.init(args=None)
    node = rclpy.create_node('image_listener')
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )
    subscription = node.create_subscription(
        Image,
        '/rgb_camera/image_raw',
        image_handler.image_callback,
        qos_profile
    )
    
    # 等待接收第一张图像
    while not image_handler.image_received:
        rclpy.spin_once(node)
    image = image_handler.current_image
    height, width = image.shape[:2]
    # 定义ROI（上半部分，去除地面影响）
    upper_half = image[0:height//2, 0:width]
    hsv = cv2.cvtColor(upper_half, cv2.COLOR_BGR2HSV)
    # 定义黄色HSV范围
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    # 创建黄色掩膜
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # 开运算去除噪声，闭运算连接区域(这里去除连接杆的影响)
    kernel = np.ones((200,200), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # 查找轮廓
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    
    # 过滤小轮廓
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 100]
    
    if not contours:
        return 1# 默认
    
    # 取最大轮廓
    largest_contour = max(contours, key=cv2.contourArea)

    # 计算黄灯中心点
    bottom_height = max(largest_contour, key=lambda pt: pt[0][1])[0][1]

    # 计算垂直FOV
    horizontal_fov = np.deg2rad(84)  # 84°（弧度）
    aspect_ratio = width / height
    vertical_fov = 2 * np.arctan(np.tan(horizontal_fov / 2) / aspect_ratio)
    
    # 计算垂直焦距
    focal_length_v = (height / 2) / np.tan(vertical_fov / 2)
    
    # 图像中心点
    center_v = height // 2
    
    # 距离计算
    distance = (focal_length_v * (light_height - camera_height)) / (center_v - bottom_height)
    node.get_logger().info(f"当前距离黄灯: {distance}m")
    # 清理资源
    node.destroy_subscription(subscription)
    rclpy.shutdown()
    return distance

def yellow_wait():
    """
    停止5s

    returns:
        int: 完成返回1
    """
    for i in range(1,6):
        print(f"黄灯等待中：{i}")
        sleep(1)
    return 1

def finish():
    """
    进行语音播报(充电中)

    returns:
        int: 37
    """
    return 4

if __name__ == '__main__':
    result = QRcode()
    print("\n----- 最终识别结果 -----")
    print(f"返回值: {result}")