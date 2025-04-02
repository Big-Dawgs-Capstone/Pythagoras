import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import PoseStamped
import cv2
from rclpy.qos import QoSProfile
from px4_offboard.msg import DeltaPose # may not even need this
# from px4_msgs.msg import OffboardControlMode
# from px4_msgs.msg import TrajectorySetpoint
# from px4_msgs.msg import VehicleStatus
# from px4_msgs.msg import VehicleAttitude
# from px4_msgs.msg import VehicleCommand

class Px4Controller(Node):
    def __init__(self):
        super().__init__('px4_autonomous_controller')

        # Publishers
        # self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        # self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        # self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        # Target position storage
        self.target = None
        self.current = None

        # Subscription
        self.sub1 = self.create_subscription(PoseStamped, '/autonomous/current', self.current_callback, 10)
        self.sub2 = self.create_subscription(PoseStamped, '/autonomous/goal', self.target_callback, 10)

    # def current_callback(self, msg: PoseStamped):
    #     self.current = msg.pose.position
    #     self.try_autonomous()

    # def target_callback(self, msg: PoseStamped):
    #     self.target = msg.pose.position
    #     self.try_autonomous()

    # def try_autonomous(self):
    #     if self.current and self.target:
    #         self.px4_autonomous(self.current, self.target)

    # def px4_autonomous(self, current: PoseStamped, target: PoseStamped):

    #     self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1., param2=6.)
    #     # Start setpoint timer
    #     self.timer = self.create_timer(0.1, self.send_setpoint)

    # def send_setpoint(self):
    #     if self.target is None or self.current is None:
    #         return

    #     # Compute distance to target
    #     dx = self.target.x - self.current.x
    #     dy = self.target.y - self.current.y
    #     dz = self.target.z - self.current.z
    #     distance = np.sqrt(dx**2 + dy**2 + dz**2)

    #     self.get_logger().info(f"Distance to goal: {distance:.2f}m")

    #     # Check if goal is reached
    #     if distance < 0.1:  # Tune as needed
    #         self.get_logger().info("Goal reached! Switching back to Manual mode.")
    #         self.target = None
    #         self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1., param2=1.)
    #         self.timer.cancel()  # Stop sending setpoints
    #         return

    #     # Send OffboardControlMode
    #     ts_us = int(self.get_clock().now().nanoseconds / 1000)
    #     offboard_mode = OffboardControlMode()
    #     offboard_mode.timestamp = ts_us
    #     offboard_mode.position = True
    #     self.publisher_offboard_mode.publish(offboard_mode)

    #     # Send TrajectorySetpoint
    #     traj = TrajectorySetpoint()
    #     traj.timestamp = ts_us
    #     traj.position[0] = self.current_target.x
    #     traj.position[1] = self.current_target.y
    #     traj.position[2] = self.current_target.z
    #     traj.velocity = [float('nan')] * 3
    #     traj.acceleration = [float('nan')] * 3
    #     traj.yaw = float('nan')
    #     traj.yawspeed = 0.0
    #     self.publisher_trajectory.publish(traj)

    # def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
    #     msg = VehicleCommand()
    #     msg.param1 = param1
    #     msg.param2 = param2
    #     msg.param7 = param7
    #     msg.command = command
    #     msg.target_system = 1
    #     msg.target_component = 1
    #     msg.source_system = 1
    #     msg.source_component = 1
    #     msg.from_external = True
    #     msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    #     self.vehicle_command_publisher_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = Px4Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()