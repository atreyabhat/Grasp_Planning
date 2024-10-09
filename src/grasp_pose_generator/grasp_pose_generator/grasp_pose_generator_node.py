#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
from transforms3d import euler
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt

class GraspPoseGenerator(Node):
    def __init__(self):
        super().__init__('grasp_pose_generator')
        
        # Subscribers
        self.grasp_rect_sub = self.create_subscription(
            Float32MultiArray,
            '/grasp_rectangle',
            self.grasp_rect_callback,
            10
        )
        self.depth_image_sub = self.create_subscription(
            Image,
            # '/realsense/depth/image_raw',
            '/vbm/depth/image_raw',
            self.depthImage_callback,
            10
        )

        # Publisher
        self.grasp_pose_pub = self.create_publisher(PoseStamped, '/grasp_pose', 10)
        
        self.grasp_rect = None
        self.depth_image = None
        self.bridge = CvBridge()

        # Camera frame
        self.camera_frame = 'lens'

    def grasp_rect_callback(self, msg):
        self.grasp_rect = msg.data
        self.get_logger().info(f"Grasp rect: {self.grasp_rect}")
        self.generate_grasp_pose()

    def depthImage_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        plt.imshow(self.depth_image)
        plt.show()
        self.generate_grasp_pose()

    def generate_grasp_pose(self):
        if self.grasp_rect is None or self.depth_image is None:
            print("Grasp rect or depth image is None")
            return

        # Extract grasp rectangle data
        center_x, center_y, width, height, angle = self.grasp_rect
        print(center_x, center_y, width, height, angle)

        # Ensure depth image is 224x224
        # self.depth_image = cv2.resize(self.depth_image, (224, 224))
        H, W = self.depth_image.shape

        self.fx = 554.256
        self.fy = 554.256
        self.cx = H//2
        self.cy = W//2

        # Calculate 3D position
        depth_value = self.depth_image[int(center_y), int(center_x)]
        x = (center_x - self.cx) * depth_value / self.fx
        y = (center_y - self.cy) * depth_value / self.fy
        z = 1.1 - depth_value

        print(x, y, z)

        # Create PoseStamped message
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = self.camera_frame
        grasp_pose.header.stamp = self.get_clock().now().to_msg()

        grasp_pose.pose.position.x = x
        grasp_pose.pose.position.y = y
        grasp_pose.pose.position.z = float(z)

        # Calculate orientation
        angle_rad = np.radians(angle)
        quat = euler.euler2quat(np.pi, 0, ((angle_rad % np.pi) - np.pi/2))
        grasp_pose.pose.orientation.x = quat[1]
        grasp_pose.pose.orientation.y = quat[2]
        grasp_pose.pose.orientation.z = quat[3]
        grasp_pose.pose.orientation.w = quat[0]

        # Publish the grasp pose
        self.grasp_pose_pub.publish(grasp_pose)
        self.get_logger().info(f"Published grasp pose: {grasp_pose.pose}")

def main(args=None):
    rclpy.init(args=args)
    grasp_pose_generator = GraspPoseGenerator()
    rclpy.spin(grasp_pose_generator)
    grasp_pose_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()