# 本程序用于实现从摄像头获取图像，并进行处理提取出导航所需要的信息
# 循线节点名为line_follower
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  #用于将ROS图像消息转换为OpenCV图像
import cv2

class LineFollower(Node): #定义一个类LineFollower，继承自Node类，用于处理图像信息，实现循线功能
    def __init__(self):
        super().__init__('line_follower') #初始化节点,节点名称为line_follower
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )   #创建一个订阅者，订阅名为/image_raw的图像消息，回调函数为image_callback，QoS为10
            # QoS(Quality of Service)是一种服务质量，用于描述消息传输的可靠性和延迟
    def image_callback(self , msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # 从ROS图像消息转换为OpenCV图像，desired_encoding='bgr8'表示转换为BGR格式
