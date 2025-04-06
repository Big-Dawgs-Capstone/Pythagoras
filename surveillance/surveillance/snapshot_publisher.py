from typing import List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.qos import QoSProfile
from yolo_msgs.msg import DetectionArray, Detection
from geometry_msgs.msg import PoseStamped
from surveillance_msgs.msg import Snapshot, SnapshotArray
from message_filters import Subscriber, Cache
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge, CvBridgeError
import cv2


class SnapshotPublisher(Node):
    """
    Detects new objects entering the environment and publishes their type, confidence, and position relative to drone.
    """

    def __init__(self):
        super().__init__(f"snapshot_publisher")
        self.get_logger().info("Starting snapshot publisher node...")

        self.bridge = CvBridge()

        self.current_detections_sub = self.create_subscription(
            DetectionArray, "/yolo/tracking", self.publish_snapshots, qos_profile=10
        )

        self.current_drone_pose_sub = Subscriber(
            self, PoseStamped, "/visual_slam/tracking/vo_pose", qos_profile=10
        )
        # ring buffer
        self.cache_drone_pose: Cache = Cache(
            self.current_drone_pose_sub, 1
        )

        self.current_fused_image = Subscriber(
            self, Image, "/fused/image_raw", qos_profile=10
        )
        self.cache_fused_image: Cache = Cache(self.current_fused_image, 1)

        self.snapshot_pub = self.create_publisher(
            SnapshotArray, "/fused/snapshots", qos_profile=10
        )

        self.prev_detections: List[Detection] = []


    def publish_snapshots(self, curr_detections: DetectionArray):
        pose = self.cache_drone_pose.getLast()

        if pose is None:
            return

        new_detections = self.get_new_detections(
            curr_detections.detections, self.prev_detections
        )
        self.prev_detections = curr_detections.detections

        # produce snapshots
        snaps = self.produce_snapshots(pose, new_detections)

        if snaps.snapshots == []:
            print("No new objects entered scene...")
            return

        self.snapshot_pub.publish(snaps)

    def convert_imgmsg_to_jpeg(self, obj_img: Image) -> CompressedImage:

        try:
            cv_image = self.bridge.imgmsg_to_cv2(obj_img, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error("CvBridge error: %s", e)
            return

        ret, buffer = cv2.imencode(".jpg", cv_image)
        if not ret:
            self.get_logger().error("Object image compression to jpeg failed...")
            return

        comp_img_msg = CompressedImage()
        comp_img_msg.header = obj_img.header
        comp_img_msg.format = "jpeg"
        comp_img_msg.data = np.array(buffer).tobytes()

        return comp_img_msg

    def produce_snapshots(
        self, pose: PoseStamped, detections: DetectionArray
    ) -> SnapshotArray:
        
        obj_img = self.cache_fused_image.getLast()

        snap_array: SnapshotArray = SnapshotArray()
        snaps = []
        for det in detections:
            snap = Snapshot()
            snap.detection_time = Time()
            snap.class_object = det.class_name
            snap.conf_object = det.score
            snap.image_object = self.convert_imgmsg_to_jpeg(obj_img)
            snap.drone_pose = pose
            snaps.append(snap)
        snap_array.snapshots = snaps
        return snap_array

    def get_new_detections(
        self, curr_detections: List[Detection], prev_detections: List[Detection]
    ) -> DetectionArray:

        curr_det_ids = [det.class_id for det in curr_detections]
        prev_det_ids = [det.class_id for det in prev_detections]

        # considers only the new detections, not old ones leaving the scene
        new_det_ids = list(set(curr_det_ids) - set(prev_det_ids))
        new_detections = []
        for det in curr_detections:
            if det.class_id in new_det_ids:
                new_detections.append(det)

        return new_detections


def main(args=None):
    rclpy.init(args=args)
    node = SnapshotPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()