#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion

# Rotation from body to camera
R_body_to_camera = np.array([
    [0.99756421, 0.04327479, -0.05470779],
    [-0.04392193, 0.99897786, -0.01068201],
    [0.0541896, 0.01305886, 0.99844527]
])
R_camera_to_body = R_body_to_camera.T

def quaternion_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*(y**2 + z**2),     2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x**2 + z**2),     2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
    ])

def matrix_to_quaternion(R):
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2,1] - R[1,2]) / S
        y = (R[0,2] - R[2,0]) / S
        z = (R[1,0] - R[0,1]) / S
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
        w = (R[2,1] - R[1,2]) / S
        x = 0.25 * S
        y = (R[0,1] + R[1,0]) / S
        z = (R[0,2] + R[2,0]) / S
    elif R[1,1] > R[2,2]:
        S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
        w = (R[0,2] - R[2,0]) / S
        x = (R[0,1] + R[1,0]) / S
        y = 0.25 * S
        z = (R[1,2] + R[2,1]) / S
    else:
        S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
        w = (R[1,0] - R[0,1]) / S
        x = (R[0,2] + R[2,0]) / S
        y = (R[1,2] + R[2,1]) / S
        z = 0.25 * S
    return np.array([x, y, z, w])

class BodyPoseTransformer(Node):
    def __init__(self):
        super().__init__('body_pose_transformer')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/visual_slam/tracking/vo_pose',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        self.get_logger().info("Listening to /visual_slam/tracking/vo_pose and publishing to /mavros/vision_pose/pose")

    def listener_callback(self, msg: PoseStamped):
        # Translation
        t_wc = np.array([msg.pose.position.x,
                         msg.pose.position.y,
                         msg.pose.position.z])

        # Rotation
        q_wc = [msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w]
        R_wc = quaternion_to_matrix(q_wc)

        # Transform
        R_wb = R_wc @ R_camera_to_body
        t_wb = t_wc  # (apply translation offset here if needed)

        q_wb = matrix_to_quaternion(R_wb)

        body_pose = PoseStamped()
        body_pose.header = msg.header
        body_pose.pose.position.x = t_wb[0]
        body_pose.pose.position.y = t_wb[1]
        body_pose.pose.position.z = t_wb[2]
        body_pose.pose.orientation = Quaternion(
            x=q_wb[0], y=q_wb[1], z=q_wb[2], w=q_wb[3]
        )

        self.publisher.publish(body_pose)

def main(args=None):
    rclpy.init(args=args)
    node = BodyPoseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

