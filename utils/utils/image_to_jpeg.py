import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image, CompressedImage
import numpy as np

class CompressedImageNode(Node):

    def __init__(self):
        super().__init__(f'compressor_node')

        self.declare_parameter('topic', value='')
        self.topic : str = self.get_parameter('topic').get_parameter_value().string_value
        
        try:
            self._check_valid_topic(self.topic)
        except ValueError as e:
            print(e)
            exit(1)

        self.jpeg_topic = f'/jpeg{self.topic}'

        self.raw_stream = self.create_subscription( Image, self.topic, self.convert_imgmsg_to_jpeg, qos_profile=10)
        #/jpeg before ro after - prob after but this is easier to see
        self.jpeg_stream = self.create_publisher(CompressedImage, self.jpeg_topic, qos_profile=10)

        self.bridge = CvBridge()

    def convert_imgmsg_to_jpeg(self, msg : Image) -> CompressedImage:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print("CvBridge error: %s", e)
            return
        
        ret, buffer = cv2.imencode('.jpg', cv_image)
        if not ret:
            print("Object image compression to JPG failed")
            return

        comp_img_msg = CompressedImage()
        comp_img_msg.header = msg.header
        comp_img_msg.format = "jpeg"
        comp_img_msg.data = np.array(buffer).tobytes()

        self.jpeg_stream.publish(comp_img_msg)

        print(f'published jpeg img converted from {self.topic} to {self.jpeg_topic}')

    def _check_valid_topic(self, topic : str):
        if topic == '':
                raise ValueError('No topic name provided...')
        topics = self.get_topic_names_and_types()
        for name, types in topics:
            if name == topic and types[0] != 'sensor_msgs/msg/Image':
                raise ValueError('Topic must publish messages of type sensor_msg/Image...')

def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()