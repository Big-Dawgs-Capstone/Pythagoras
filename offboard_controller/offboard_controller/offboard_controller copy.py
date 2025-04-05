import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import Image
from msg import OffboardControlMode, TrajectorySetpoint
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import PoseStamped
import cv2
from rclpy.qos import QoSProfile

tolerance = 0
class OffboardController(Node):
    def __init__(self):
        super().__init__('Offboard Controller')

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        # Subscriptions
        self.target_xyz_sub = self.create_subscription(PoseStamped, '/goal', self.target_callback, 10)
        self.autonomous_toggle_sub = self.create_subscription(bool,' /cancel', self.killswitch_callback, 10)

        # Target pos
        self.target_x = None
        self.target_y = None
        self.target_z = None

        # Timer for Offboard message - must be received by px4 @ atleast 2Hz
        self.timer = self.create_timer(0.2, self.publish_offboard_callback)

        # bool to track autonomous on/off
        self.is_auto_on = False

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()

        msg.position = np.array([self.target_x, self.target_y, self.target_z], dtype=np.float32)
        msg.yaw = -3.14; 

        msg.timestamp = self.get_clock().now().nanoseconds() // 1000

        self.publisher.publish(msg)
        self.get_logger().info('Trajectory Setpoint message published')

    def target_callback(self, msg: PoseStamped):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.target_z = msg.pose.position.zπ

        self.is_auto_on = True
        self.publish_trajectory_setpoint()

    def killswitch_callback(self, msg: bool):
         self.is_auto_on = False

    def publish_offboard_callback(self):
        if self.is_auto_on:
          msg = OffboardControlMode()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False

        msg.timestamp = self.get_clock().now().nanoseconds() // 1000

        self.publisher.publish(msg)
        self.get_logger().info('Offboard control mode message published')

    

def main(args=None):
    rclpy.init(args=args)
    node = OffboardController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
