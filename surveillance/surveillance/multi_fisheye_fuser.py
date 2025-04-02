import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.qos import QoSProfile


class MultiFisheyeFuser(Node):
    def __init__(self):
        self.log_counter = 0
        print('Starting fisheye fuser node...')
        super().__init__(f'fisheye_fuser')

        self.bridge = CvBridge()

        self.sub1_fisheye = Subscriber(self, Image, '/fisheye/camera_0/image_raw', qos_profile=10)
        self.sub2_fisheye = Subscriber(self, Image, '/fisheye/camera_1/image_raw', qos_profile=10)
        # ideal -> Subscriber(self, Image, '/camera/camera/color/image_raw', qos_profile=10)
        self.sub3_realsense = Subscriber(self, Image, '/camera/infra1/image_rect_raw', qos_profile=10)

        self.fused_img_pub = self.create_publisher(Image, '/fused/image_raw', qos_profile=10)

        # image synchronizer with slop tolerance (in seconds)
        # if node gets four images from four topics within 50ms, it's good to 
        # trigger callback and fuse images
        self.ts = ApproximateTimeSynchronizer(
            [self.sub1_fisheye, self.sub3_realsense, self.sub2_fisheye],
            queue_size=10,
            slop=1,
            allow_headerless=False
        )

        self.ts.registerCallback(self.fuse_images)


    def fuse_images(self, img0, img1, img2):
        self.log_counter+=1
        print(f"Fuse image callback triggered {self.log_counter}th time...")
        try:
            # resize and fuse images
            cv_imgs = [self.bridge.imgmsg_to_cv2(img, 'bgr8') for img in [img0, img1, img2]]
            h, w = cv_imgs[0].shape[:2]
            cv_imgs = [cv2.resize(img, (w, h)) for img in cv_imgs]
            fused = np.hstack(cv_imgs)

            # cv2.imshow('Fused Image', fused)
            # cv2.waitKey(1)
            
            # publish that shit
            fused = self.bridge.cv2_to_imgmsg(fused, encoding='bgr8')
            self.fused_img_pub.publish(fused)
        except Exception as e:
            print(f"Error in fuse_images: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MultiFisheyeFuser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()