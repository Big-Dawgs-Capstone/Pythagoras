import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg

class PointCloudAggregator(Node):
    def __init__(self):
        super().__init__('pointcloud_aggregator')

        self.pc_publisher = self.create_publisher(PointCloud2, '/aggregated_map', 10)
        self.timer = self.create_timer(1.0, self.publish_map)

        self.subscription_pc = self.create_subscription(
            PointCloud2,
            '/depth/color/points',
            self.pointcloud_callback,
            10)

        self.subscription_pose = self.create_subscription(
            PoseStamped,
            '/visual_slam/tracking/vo_pose',
            self.pose_callback,
            10)

        self.latest_pose = None
        self.global_map = o3d.geometry.PointCloud()

        self.get_logger().info("Aggregator node started.")

    def publish_map(self):
        if len(self.global_map.points) == 0:
            return

        points = np.asarray(self.global_map.points)

        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        msg = pc2.create_cloud_xyz32(header, points)
        self.pc_publisher.publish(msg)

    def pose_callback(self, msg: PoseStamped):
        position = msg.pose.position
        orientation = msg.pose.orientation
        self.latest_pose = {
            'position': np.array([position.x, position.y, position.z]),
            'orientation': np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        }

    def pointcloud_callback(self, msg: PointCloud2):
        if self.latest_pose is None:
            return

        # Convert ROS PointCloud2 to XYZ points
        points = np.array([
            [p[0], p[1], p[2]] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ])

        if points.size == 0:
            return

        # Convert to Open3D format
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)

        # Transform to world frame using current pose
        transformed_pc = self.transform_pointcloud(pc, self.latest_pose)

        # Accumulate into global map
        self.global_map += transformed_pc
        self.global_map = self.global_map.voxel_down_sample(voxel_size=0.10)

        self.get_logger().info(f"Aggregated {len(points)} points, total: {len(self.global_map.points)}")

    def transform_pointcloud(self, pc, pose):
        rotation = Rotation.from_quat(pose['orientation']).as_matrix()
        translation = pose['position'].reshape((3, 1))
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = translation.flatten()
        
        return pc.transform(transform)

def main(args=None):
    rclpy.init(args=args)
    aggregator = PointCloudAggregator()
    rclpy.spin(aggregator)
    aggregator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
