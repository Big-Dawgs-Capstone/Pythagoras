import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.qos import QoSProfile


class MultiCameraFuser(Node):
    def __init__(self):
        super().__init__(f"multi_camera_fuser_node")

        self.get_logger().info("Fisheye fuser node has started...")

        self.bridge = CvBridge()

        self.fisheye_sub1 = Subscriber(
            self, Image, "/fisheye/camera_0/image_raw", qos_profile=10
        )
        self.fisheye_sub2 = Subscriber(
            self, Image, "/fisheye/camera_1/image_raw", qos_profile=10
        )
        # '/camera/camera/color/image_raw'
        self.realsense_sub = Subscriber(self, Image, "/left/image_rect", qos_profile=10)

        self.fused_img_pub = self.create_publisher(
            Image, "/fused/image_raw", qos_profile=10
        )

        # software synchronizing incoming frames from different cameras
        # if node receives frames from relevant topics within x sec of each other, trigger callback
        self.ts = ApproximateTimeSynchronizer(
            [self.fisheye_sub1, self.realsense_sub, self.fisheye_sub2],
            queue_size=10,
            slop=1,
            allow_headerless=False,
        )

        self.ts.registerCallback(self.fuse_images)

        self.log_counter = 0

    def fuse_images(self, img0, img1, img2):
        """Fuses images together. 
        @Impl current implementation is just stacking the images side-by-side.

        Args:
            img0 (Image): Fisheye Image 1
            img1 (Image): Realsense Image 2
            img2 (Image): Fisheye Image 3
        """

        self.log_counter += 1
        self.get_logger().info(f"Fusing images {self.log_counter}th time...")

        try:
            # convert all ros2 images to cv2 images
            cv_imgs = [
                self.bridge.imgmsg_to_cv2(img, "bgr8") for img in [img0, img1, img2]
            ]
            # make sure all images have the same shape
            h, w = cv_imgs[0].shape[:2]
            # stack images side by side
            cv_imgs = [cv2.resize(img, (w, h)) for img in cv_imgs]
            fused = np.hstack(cv_imgs)

            fused = self.bridge.cv2_to_imgmsg(fused, encoding="bgr8")
            self.fused_img_pub.publish(fused)
        except Exception as e:
            self.get_logger().error(f"Error when fusing images: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraFuser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
