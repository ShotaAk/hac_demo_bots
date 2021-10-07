#!/usr/bin/env python3
# coding: UTF-8

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class Recognition(Node):

    def __init__(self):
        super().__init__('recognition_node')
        self._sub_image = self.create_subscription(Image, 'image_raw', self.image_callback, 1)

        qos_profile = QoSProfile(depth=1)
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE

        self._pub_text = self.create_publisher(String, 'text', 1)
        self._pub_result_image = self.create_publisher(Image, 'result_image', qos_profile)

        self._polling_timer = self.create_timer(1, self.timer_callback)
        self._bridge = CvBridge()

        self.get_logger().info('start')

    def timer_callback(self):
        text = String()
        text.data = "Hello world!"
        self._pub_text.publish(text)

    def image_callback(self, msg):
        cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        result_image = cv_image
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower = np.array([9, 100, 100])
        upper = np.array([29, 255, 255])

        hsv_mask = cv2.inRange(hsv_image, lower, upper)
        result_image = cv2.bitwise_and(cv_image, cv_image, mask=hsv_mask)

        # ボール検出

        # パネル検出

        # 自己位置補正

        # 処理後の画像を出力
        result_ros_msg = self._bridge.cv2_to_imgmsg(result_image, 'bgr8')
        self._pub_result_image.publish(result_ros_msg)
        # self._pub_result_image.publish(
        #     self._bridge.cv2_to_imgmsg(result_image, 'bgr8'))

        text = String()
        text.data = "Hello world!"
        self._pub_text.publish(text)

def main(args=None):
    rclpy.init(args=args)

    node = Recognition()

    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
