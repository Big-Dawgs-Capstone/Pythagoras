#########################
# Author: Mohandeep Kapur
#########################

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class FisheyeCameraView(Node):
    """
    Renders stream of Fisheye Camera sensor_msg/Image data.
    """

    def __init__(self):
        super().__init__(f'fisheye_camera_view')

        self.bridge = CvBridge()

        self.declare_parameter('topic_num', value=-1)
        topic_num = self.get_parameter('topic_num').get_parameter_value().integer_value

        if topic_num < 0:
            print('Invalid topic number given... must be non-negative...')
            exit(1)
        
        self.subscription = self.create_subscription(
            Image,
            f'/fisheye/camera_{topic_num}/image_raw', 
            self.listener_callback,
            10
        )

        self.get_logger().info("Image subscriber node has been started!")

    def listener_callback(self, msg):
        try:
            # convert ros2 Image to CV2 image type
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # display opencv image
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)  # 1ms delay for the OpenCV window to refresh
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FisheyeCameraView()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()