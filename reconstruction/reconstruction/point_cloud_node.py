import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
# why import not resolved here but pkg found
from realsense2_camera_msgs.msg import RGBD  
import numpy as np
import open3d as o3d
from cv_bridge import CvBridge
from collections import deque

# averaging not necessary, FPS of RGBD topic is literally 12
image_batch_size = 1

class PointCloudNode(Node):
    def __init__(self):
        super().__init__("point_cloud_node")

        self.bridge = CvBridge()
        self.average_num = image_batch_size

        self.intrinsics = None

        self.color_buffer = deque(maxlen=self.average_num)
        self.depth_buffer = deque(maxlen=self.average_num)

        self.rgbd_sub = self.create_subscription(
            RGBD, "/camera/camera/rgbd", self.accum_rgbd, 10
        )

        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()

        view_control = self.vis.get_view_control()
        # Set the view angle (camera position and orientation)
        # You can change the values to adjust the camera's position
        view_control.set_front([1, 0, 0])  # Front view vector (X-axis)
        view_control.set_up([0, 1, 0])     # Up vector (Y-axis)
        view_control.set_lookat([0, 0, 0]) # Look at the center of the scene

        # self.point_cloud_pub = self.create_publisher(
        # )

    def accum_rgbd(self, msg):
        """Accumulate RGBD snapshots"""

        color_image = self.bridge.imgmsg_to_cv2(msg.rgb, desired_encoding="bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(msg.depth, desired_encoding="16UC1")
        self.color_buffer.append(color_image)
        self.depth_buffer.append(depth_image)

        if not self.intrinsics:
            self.intrinsics = self.camera_info_to_intrinsics(msg.rgb_camera_info)

        print(len(self.color_buffer))
        if len(self.color_buffer) == self.average_num and len(self.depth_buffer) == self.average_num:
            self.rgbd_images_to_pointcloud()
    
    def rgbd_images_to_pointcloud(self):
        """ Convert RGBD inputs to point cloud """

        color_input = np.mean(np.array(self.color_buffer), axis=0).astype(np.uint8)
        depth_input = np.mean(np.array(self.depth_buffer), axis=0).astype(np.uint8)

        # rest of this is just from open3d documentation
        o3d_color = o3d.geometry.Image(color_input)
        o3d_depth = o3d.geometry.Image(depth_input)

        # NOTE we need to confirm the depth scale of our real sense.
        # depth_scale is 0.001m for all 400 series cameras as conf by MartyG 2 yrs ago
        # this function wants inverse of that
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d_color, o3d_depth, depth_scale=1000, depth_trunc=3.0, convert_rgb_to_intensity=False
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, self.intrinsics)

        # needs to be flipped 180 deg for some reason, per documentation 
        pcd.transform([[1, 0,  0, 0],
                       [0, -1, 0, 0],  # Flip Y-axis
                       [0, 0, -1, 0],  # Flip Z-axis
                       [0, 0,  0, 1]])

        self.vis.clear_geometries()
        self.vis.add_geometry(pcd)
        self.vis.poll_events()
        self.vis.update_renderer()

        self.color_buffer.clear()
        self.depth_buffer.clear()
    
    def camera_info_to_intrinsics(self, camera_info : CameraInfo) -> o3d.camera.PinholeCameraIntrinsic:

        width = camera_info.width
        height = camera_info.height
        fx = camera_info.k[0]  # Focal length x
        fy = camera_info.k[4]  # Focal length y
        cx = camera_info.k[2]  # Principal point x
        cy = camera_info.k[5]  # Principal point y
        
        return o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
