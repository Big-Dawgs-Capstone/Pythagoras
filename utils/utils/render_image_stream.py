import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImageStreamView(Node):

    def __init__(self):
        super().__init__(f'debug_boxes')

        self.declare_parameter('topic', value='')
        topic : str = self.get_parameter('topic').get_parameter_value().string_value
        
        try:
            self._check_valid_topic(topic)
        except ValueError as e:
            print(e)
            exit(1)

        self.bridge = CvBridge()
        self.create_subscription(Image, f'/yolo/dbg_image', self.listener_callback, qos_profile=10)

    def listener_callback(self, msg):
        try:
            # convert ros2 Image to CV2 image type
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # display opencv image
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def _check_valid_topic(self, topic : str):
        if topic is '':
                raise ValueError('No topic name provided...')
        topics = self.get_topic_names_and_types()
        for name, types in topics:
            if name == topic and types[0] != 'sensor_msgs/msg/Image':
                raise ValueError('Topic must publish messages of type sensor_msg/Image...')

def main(args=None):
    rclpy.init(args=args)
    node = ImageStreamView()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()