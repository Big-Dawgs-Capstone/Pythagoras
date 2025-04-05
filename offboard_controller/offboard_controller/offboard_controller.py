import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import Image
from msg import OffboardControlMode, TrajectorySetpoint
from mavros_msgs.srv import CommandBool, SetMode
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import PoseStamped
import cv2
from rclpy.qos import QoSProfile

tolerance = 0.
class OffboardController(Node):
    def __init__(self):
        super().__init__('Offboard Controller')
        # Publishers
        # self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/mavros/setpoint_position/local', 10)

        # Subscriptions
        self.target_xyz_sub = self.create_subscription(PoseStamped, '/goal', self.target_callback, 10)
        self.autonomous_toggle_sub = self.create_subscription(bool,' /cancel', self.killswitch_callback, 10)

        # Target pos
        self.target = None

        # Timer for Trajectory message - must be received by mavros @ atleast 2Hz BEFORE it switches to offboard mode
        self.timer = self.create_timer(0.1, self.publish_trajectory_setpoint)

        # bool to track autonomous on/off
        self.is_auto_on = False
        self.transformation_mat = np.array([
                                            [  0.99756421,  0.04327479,  -0.05470779],
                                            [ -0.04392193,  0.99897786,  -0.01068201],
                                            [  0.05411896,  0.01305886,   0.99844527]
                                        ])

    def publish_trajectory_setpoint(self):
        self.get_logger.info("Setting msg to target")
        
        msg = self.target
        msg.header.stamp = self.get_clock().now().nanoseconds() // 1000
        self.get_logger().info("Try publishing trajectory setpoint message")
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info('Trajectory Setpoint message published')

    def target_callback(self, msg: PoseStamped):
        self.get_logger().info("Target Callback Triggered")
        self.target = msg
        targ_position_raw = np.array([self.target.pose.position.x, self.target.pose.position.y, self.target.pose.position.z])
        targ_position_cooked = self.transformation_mat @ targ_position_raw
        self.target.pose.position.x = targ_position_cooked[0]
        self.target.pose.position.y = targ_position_cooked[1]
        self.target.pose.position.z = targ_position_cooked[2]
        self.is_auto_on = True
        # Sleep for a second so that mavros reqs are met
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
        # Set Mavros mode to active
        self.get_logger().info('Try to set Mavros mode to Offboard')
        moder = self.create_client(SetMode, '/mavros/set_mode')
        while not moder.wait_for_service(timeout_sec=1.0):
            self.get_logger.warn("Waiting for set_mode")
        mode_req = Setmode.Request()
        mode_req.custom_mode = "OFFBOARD"
        mode_future = moder.call_asnyc(mode_req)
        rclpy.spin_until_future_complete(self, mode_future)
        if mode_future.result().mode_sent:
            self.get_logger().info("OFFBOARD mode set successfully.")
        else:
            self.get_logger().error("Failed to set OFFBOARD mode.")

    def killswitch_callback(self, msg: bool):
        self.get_logger().info("Killswitch Callback Triggered")
        self.get_logger().ingo("Try to set mavros mode to manual")
        mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for set_mode service...")
        mode_req = SetMode.Request()
        mode_req.custom_mode = 'MANUAL'  # or 'STABILIZED', 'POSCTL', etc.
        mode_future = mode_client.call_async(mode_req)
        rclpy.spin_until_future_complete(self, mode_future)
        if mode_future.result().mode_sent:
            self.get_logger().info("Switched back to RC mode successfully.")
        else:
            self.get_logger().error("Failed to switch back to RC mode.")


    # def publish_offboard_callback(self):
    #     if self.is_auto_on:
    #       msg = OffboardControlMode()

    #     msg.position = True
    #     msg.velocity = False
    #     msg.acceleration = False
    #     msg.attitude = False
    #     msg.body_rate = False
    #     msg.thrust_and_torque = False
    #     msg.direct_actuator = False

    #     msg.timestamp = self.get_clock().now().nanoseconds() // 1000

    #     self.publisher.publish(msg)
    #     self.get_logger().info('Offboard control mode message published')

    

def main(args=None):
    rclpy.init(args=args)
    node = OffboardController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
