#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Vector3
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

def draw_cross(img, center, color=(255, 0, 0), thickness=1, size=10):
    """在指定中心位置绘制十字标记"""
    x, y = center
    cv2.line(img, (x - size, y), (x + size, y), color, thickness)
    cv2.line(img, (x, y - size), (x, y + size), color, thickness)
    cv2.putText(img, "rect", (x - 5, y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

def threshold_image(image, threshold_value=127, max_value=255, method=cv2.THRESH_BINARY):
    """对图像进行二值化处理"""
    _, thresh = cv2.threshold(image, threshold_value, max_value, method)
    return thresh

def calculate_perimeter(cnt):
    """计算轮廓的周长"""
    return cv2.arcLength(cnt, True)

def is_rectangle_like(cnt, approx, min_aspect_ratio=3.7, max_aspect_ratio=7.7, min_fill_ratio=0.5): # aspect_ratio≈5.7
    """检查轮廓是否接近矩形"""
    if len(approx) != 4:
        return False

    x, y, w, h = cv2.boundingRect(approx)
    aspect_ratio = float(w) / h
    if not (min_aspect_ratio <= aspect_ratio <= max_aspect_ratio):
        return False

    area = cv2.contourArea(cnt)
    rect_area = w * h
    fill_ratio = area / rect_area

    if fill_ratio < min_fill_ratio:
        return False

    return True

def filter_regions(image, min_area, max_area):
    """过滤区域，根据面积和形状特征筛选矩形轮廓"""
    rectangles_info = []  # 存储每个矩形的中心坐标和尺寸
    contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        perimeter = calculate_perimeter(cnt)

        # 根据面积范围筛选矩形轮廓
        if max_area >= area >= min_area:
            approx = cv2.approxPolyDP(cnt, 0.04 * perimeter, True)
            if is_rectangle_like(cnt, approx):
                x, y, w, h = cv2.boundingRect(approx)
                center = (x + w // 2, y + h // 2)
                rectangles_info.append((center, w, h))  # 添加到列表中
    return rectangles_info, contours  # 返回矩形信息和轮廓

def process_image(image):
    """处理图像并绘制矩形和十字标记"""
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # 转为灰度图
    thresh = threshold_image(gray_image, 127, 255, cv2.THRESH_BINARY_INV)
    rectangles_to_draw, contours = filter_regions(thresh, min_area=50, max_area=2000)

    # 遍历所有检测到的矩形并绘制
    for center, width, height in rectangles_to_draw:
        cv2.rectangle(image, (center[0] - width // 2, center[1] - height // 2),
                      (center[0] + width // 2, center[1] + height // 2), (0, 255, 0), 2)
        draw_cross(image, center, thickness=2, size=10)

    # 绘制所有轮廓
    cv2.drawContours(image, contours, -1, (255, 0, 0), 1)  # 用蓝色绘制所有轮廓

    if len(rectangles_to_draw) == 0:
        print("No rectangles detected.")

    # 显示处理后的图像
    cv2.imshow('Processed Image', image)
    cv2.waitKey(1)

    return image, rectangles_to_draw

def calculate_rectangle_normal(depth_image, rectangle_corners):
    """计算矩形平面的法线方向"""
    # 提取矩形角点的深度信息
    depth_points = [depth_image[corner[1], corner[0]] for corner in rectangle_corners]
    
    # 将深度信息转换为三维坐标
    fx = fy = 500  # 假设相机内参
    cx = depth_image.shape[1] // 2
    cy = depth_image.shape[0] // 2
    points_3d = []
    for (x, y), depth in zip(rectangle_corners, depth_points):
        Z = depth
        X = (x - cx) * Z / fx
        Y = (y - cy) * Z / fy
        points_3d.append([X, Y, Z])
    
    # 使用SVD计算平面法线
    points_3d = np.array(points_3d)
    centroid = np.mean(points_3d, axis=0)
    points_centered = points_3d - centroid
    U, S, Vt = np.linalg.svd(points_centered)
    normal = Vt[-1]
    return normal

def image_to_camera_coordinates(depth_image, center, fx, fy, cx, cy):
    """将图像坐标转换为相机坐标系下的三维坐标"""
    x, y = center
    depth = depth_image[y, x]
    Z = depth
    X = (x - cx) * Z / fx
    Y = (y - cy) * Z / fy
    return np.array([X, Y, Z])

class ImageRectDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/xv_sdk/xv_dev/fisheye_cameras/left/image", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/xv_sdk/xv_dev/fisheye_cameras/left/depth", Image, self.depth_callback)
        self.center_pub = rospy.Publisher("/detected_rectangle_center", Point, queue_size=10)
        self.normal_pub = rospy.Publisher("/detected_rectangle_normal", Vector3, queue_size=10)
        self.depth_image_buffer = {}
        self.fx = 500  # 假设相机内参
        self.fy = 500
        self.cx = 320  # 假设图像中心
        self.cy = 240

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        processed_image, rectangles_to_draw = process_image(cv_image)
        
        if len(rectangles_to_draw) == 1:
            center, width, height = rectangles_to_draw[0]

            # 计算矩形角点
            rectangle_corners = [
                (center[0] - width // 2, center[1] - height // 2),
                (center[0] + width // 2, center[1] - height // 2),
                (center[0] + width // 2, center[1] + height // 2),
                (center[0] - width // 2, center[1] + height // 2)
            ]

            # 获取深度图像
            depth_image = self.get_depth_image(data.header.stamp)
            if depth_image is None:
                return

            # 计算矩形法线
            normal = calculate_rectangle_normal(depth_image, rectangle_corners)
            normal_msg = Vector3()
            normal_msg.x, normal_msg.y, normal_msg.z = normal
            self.normal_pub.publish(normal_msg)

            # 计算矩形中心点的三维坐标
            center_3d = image_to_camera_coordinates(depth_image, center, self.fx, self.fy, self.cx, self.cy)
            center_3d_msg = Point()
            center_3d_msg.x, center_3d_msg.y, center_3d_msg.z = center_3d
            self.center_pub.publish(center_3d_msg)
        elif len(rectangles_to_draw) > 1:
            print("Multiple rectangles detected. Waiting for single rectangle detection.")
        else:
            print("No rectangles detected. Waiting for single rectangle detection.")

    def depth_callback(self, data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)
            return

        # 缓存深度图像及其时间戳
        self.depth_image_buffer[data.header.stamp] = depth_image

    def get_depth_image(self, timestamp):
        # 查找最接近指定时间戳的深度图像
        closest_timestamp = min(self.depth_image_buffer.keys(), key=lambda t: abs(t - timestamp))
        if abs(closest_timestamp - timestamp).to_sec() > 0.05:  # 假设时间戳差异在0.05秒内是可以接受的
            return None
        return self.depth_image_buffer.pop(closest_timestamp)

def main():
    rospy.init_node('image_rect_detector', anonymous=True)
    detector = ImageRectDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()