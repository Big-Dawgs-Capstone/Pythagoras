import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class RenderBoundingYoloBoxes(Node):

    def __init__(self):
        super().__init__(f'debug_boxes')
        self.bridge = CvBridge()
        self.create_subscription(Image, f'/yolo/dbg_image', self.listener_callback, qos_profile=10)

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
    node = RenderBoundingYoloBoxes()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


        
