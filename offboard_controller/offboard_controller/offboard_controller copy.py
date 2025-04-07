import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from msg import OffboardControlMode, TrajectorySetpoint
from mavros_msgs.srv import CommandBool, SetMode
from message_filters import Subscriber, ApproximateTimeSynchronizer
from fused.msgs import Snapshot
from cv_bridge import CvBridge
from yolo_msgs.msg import DetectionArray, BoundingBox3D
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, Vector3
import cv2
from rclpy.qos import QoSProfile

tolerance = 0.05
safety_dist = 1

class WaypointNode:
    def __init__(self, Pose):
        self.pose = Pose()
        self.next = None
        self.head = False

class OffboardController(Node):
    def __init__(self):
        super().__init__('Offboard Controller')
        # Publishers
        self.trajectory_setpoint_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.local_pose_publisher = self.create_publisher(PoseStamped, '/mavros/vision/pose', 10)
        # Subscriptions
        self.target_xyz_sub = self.create_subscription(Snapshot, '/goal', self.autonomous_phase_one, 10)
        self.autonomous_toggle_sub = self.create_subscription(bool,' /cancel', self.killswitch_callback, 10)
        # This is reading from a bogus topic not sure what the actual one will be
        self.yolo3d_detection_sub = self.create_subscription(DetectionArray, 'yolo3d/tracking', self.detections3d_cb, 10)
        self.yolo_detection_sub = self.create_subscription(DetectionArray, 'yolo/tracking', self.detections_cb, 10)
        self.orangepose_sub = self.create_subscription(PoseStamped, '/visual_slam/pose/v0_pose', self.update_orangepose_cb, 10)
        self.fusedimg_sub = self.create_subscription(Image, '/fused/image', self.update_fusedimg_cb, 10)
        # Target pos
        self.target_pos = Pose()
        self.class_name = ''
        # Oranges pose
        self.orangepose = Pose()
        # Fused image we are seeing
        self.fusedimg = Image()
        # Waypoints
        self.object_center = Pose()
        self.object_dims = Vector3()
        self.primary_target = Pose()
        self.secondary_target = Pose()
        # Detectons
        self.detections = DetectionArray()
        self.detections3d = DetectionArray()
        # Timers to publish to mavros
        self.target_timer = self.create_timer(0.1, self.publish_trajectory_setpoint)
        self.local_timer  = self.create_timer(0.1, self.publish_local_pose)
        # bool to track autonomous on/off
        self.is_auto_on = False
        self.did_phase_two = False
        # Transformation matrix via abstacle brain
        self.transformation_mat = np.array([
                                            [  0.99756421,  0.04327479,  -0.05470779],
                                            [ -0.04392193,  0.99897786,  -0.01068201],
                                            [  0.05411896,  0.01305886,   0.99844527]
                                        ])

    ########################################################################################################
    # HELPER FUNCTIONS
    ########################################################################################################
    def ray_cyl_intersect(self, direction, pose, cyl_cent, cyl_rad):
        
        dx, dy = direction[0], direction[1]
        ox, oy = pose[0] - cyl_cent[0], pose[1] - cyl_cent[1]

        a = dx**2 + dy**2
        b = 2 * (ox*dx + oy*dy)
        c = ox**2 + oy**2 - cyl_rad**2

        discriminant = b**2 - 4*a*c

        if discriminant < 0:
            return None
        elif discriminant == 0:
            t = -b / (2*a)
        else:
            sqrt_disc = np.sqrt(discriminant)
            t1 = (-b - sqrt_disc) / (2*a)
            t2 = (-b + sqrt_disc) / (2*a)
            t = min(t1, t2)  # Entry point is the smaller t

        
        if t < 0:
            return None
        
        intersection_point = pose + t*direction
        return intersection_point
    
    def transform_pose(self, raw_pose: Pose) -> Pose:
        raw_position = np.array([raw_pose.position.x,
                                 raw_pose.position.y,
                                 raw_pose.position.z])
        trans_position = self.transformation_mat @ raw_position

        q = raw_pose.pose.orientation
        siny_cosp = 2.0 * ((q.w * q.z) + (q.x * q.y))
        cosy_cosp = 1.0 - 2.0 * ((q.y * q.y) + (q.z * q.z))
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        yaw_dir = np.array([np.cos(yaw), np.sin(yaw), 0.0])

        yaw_dir_trans = self.transformation_mat @ yaw_dir
        yaw_trans = np.arctan2(yaw_dir_trans[1], yaw_dir_trans[0])

        cy = np.cos(yaw_trans * 0.5)
        sy = np.sin(yaw_trans * 0.5)
        qx, qy, qz, qw = 0.0, 0.0, sy, cy

        cooked_pose = Pose()
        cooked_pose.position.x = trans_position[0]
        cooked_pose.position.y = trans_position[1]
        cooked_pose.position.z = trans_position[2]
        cooked_pose.orientation.x = qx
        cooked_pose.orientation.y = qy
        cooked_pose.orientation.z = qz
        cooked_pose.orientation.w = qw

        return(cooked_pose)
    
    def generate_waypoints(self, num_points: int = 20):
        cx = self.object_center.position.x
        cy = self.object_center.position.y
        cz = self.object_center.position.z

        sx = self.secondary_target.position.x
        sy = self.secondary_target.position.y

        radius = np.linalg.norm(np.array([cx - sx, cy - sy]))
        start_angle = np.arctan2(sy - cy, sx - cx)
        z = self.secondary_target.position.z

        head = None
        prev = None
        for i in range(num_points):
            angle = start_angle + 2 * np.pi * i / num_points
            x = cx + radius * np.cos(angle)
            y = cy + radius * np.sin(angle)

            # Yaw to always face center
            yaw = np.arctan2(cy - y, cx - x)
            cyaw = np.cos(yaw * 0.5)
            syaw = np.sin(yaw * 0.5)

            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = syaw
            pose.orientation.w = cyaw

            node = WaypointNode(pose)
            if head is None:
                node.head = True
                head = node
            if prev:
                prev.next = node
            prev = node

        if prev:
            prev.next = head  # circular loop

        return head


    ########################################################################################################
    # CALLBACKS/SUBSCRIBERS
    ########################################################################################################
    def killswitch_callback(self, msg: bool):
        if(msg):
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

    def update_orangepose_cb(self, msg: PoseStamped):
        self.orangepose = self.transform_pose(msg)
    
    def detections_cb(self, msg: DetectionArray):
        self.detections = msg

    def detections3d_cb(self, msg: DetectionArray):
        self.detections3d = msg
    
    def update_fusedimg_cb(self, msg: Image):
        self.fusedimg = msg


    ########################################################################################################
    # AUTONOMOUS
    ########################################################################################################
    def autonomous_phase_one(self, msg: Snapshot):
        self.get_logger().info("Target Callback Triggered")
        self.primary_target = self.transform_pose(msg.drone_pose.pose)
        self.target_pos = self.transform_pose(msg.drone_pose.pose)
        self.class_name = msg.class_name
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

    def autonomous_phase_two(self):
        for obj in self.detections:
            if(obj.class_name == self.class_name):
                center2d = obj.bbox.center
                w_low = self.fusedimg.width * .33333
                w_high = self.fusedimg.width * .66666
                angle = 0.0  # initial yaw
                delta_angle = np.radians(10)  # how much to rotate per loop (10 degrees)
                r = self.get_clock().now()

                while(center2d.point.x < w_low or center2d.point.x > w_high):
                    # Increment yaw
                    angle += delta_angle
                    angle %= 2 * np.pi  # keep it in [0, 2π]

                    # Convert yaw to quaternion
                    cy = np.cos(angle * 0.5)
                    sy = np.sin(angle * 0.5)

                    # Set target pose (keep position fixed)
                    new_pose = Pose()
                    new_pose.position = self.orangepose.position  # stay at current position
                    new_pose.orientation.x = 0.0
                    new_pose.orientation.y = 0.0
                    new_pose.orientation.z = sy
                    new_pose.orientation.w = cy
                    self.target_pos = new_pose
                    # Sleep for smooth rotation (~10Hz)
                    self.get_clock().sleep_for(Duration(seconds=1.0))
        
        for obj in self.detections3d:
            if(obj.class_name == self.class_name):
                center = self.transform_pose(obj.bbox3d.center)
                self.object_center = center
                self.object_dims = obj.bbox3d.size
                orangepose = self.orangepose

        cyl_cent = np.array([center.position.x, center.position.y, center.position.z])
        pose = np.array([orangepose.position.x, orangepose.position.y, orangepose.position.z])
        direction = (cyl_cent - pose) / np.linalg.norm(cyl_cent - pose)
        w = obj.bbox3d.size[1]
        d = obj.bbox3d.size[2]
        r = max(w, d) + safety_dist

        intermediary_point = self.ray_cyl_intersect(direction=direction, pose=pose, cyl_cent=cyl_cent, cyl_rad=r)
        target_position = np.array([intermediary_point[0], intermediary_point[1], center.position.z])

        delta = cyl_cent[:2] - target_position[:2]
        yaw = np.arctan2(delta[1], delta[0])
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        qx = 0.0
        qy = 0.0
        qz = sy
        qw = cy

        self.secondary_target.position.x = target_position[0]
        self.secondary_target.position.y = target_position[1]
        self.secondary_target.position.z = target_position[2]
        self.secondary_target.orientation.x = qx
        self.secondary_target.orientation.y = qy
        self.secondary_target.orientation.z = qz
        self.secondary_target.orientation.w = qw

        self.target_pos = self.secondary_target

    def autonomous_phase_three(self):
        waypoint = self.generate_waypoints()
        circle_count = 0
        max_waits = 200

        while(circle_count <= 5):
            if(waypoint.head):
                circle_count+=1
            
            self.target_pos = waypoint.pose
            wait_counter = 0
            while(wait_counter < max_waits):
                p1 = np.array([self.orangepose.position.x, self.orangepose.position.y, self.orangepose.position.z])
                p2 = np.array([waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z])
                dist_targ = np.linalg.norm(p1 - p2)
                if dist_targ <= tolerance:
                    break
                self.get_clock().sleep_for(Duration(seconds=0.05))
                wait_counter += 1
            
            if wait_counter >= max_waits:
                self.get_logger().error("Timed out waiting to reach waypoint probs stuck")
                
            waypoint = waypoint.next

        self.target_pos = self.secondary_target

        
    ########################################################################################################
    # PUBLISHERS
    ########################################################################################################
    def publish_local_pose(self):
        msg = PoseStamped()
        msg.pose = self.orangepose
        msg.header.stamp =  self.get_clock().now().nanoseconds() // 1000
        self.local_pose_publisher.publish(msg)
        
    def publish_trajectory_setpoint(self):
        self.get_logger.info("Setting msg to target")
        msg = PoseStamped()
        msg.pose = self.target_pos
        msg.header.stamp = self.get_clock().now().nanoseconds() // 1000
        self.get_logger().info("Try publishing trajectory setpoint message")
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info('Trajectory Setpoint message published')
        if(self.orangepose == self.primary_target and not self.did_phase_two):
            self.autonomous_phase_two()
        if(self.orangepose == self.secondary_target and self.did_phase_two):
            self.autonomous_phase_three()


def main(args=None):
    rclpy.init(args=args)
    node = OffboardController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
