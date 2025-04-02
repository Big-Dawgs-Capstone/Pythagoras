from typing import List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.qos import QoSProfile
from surveillance.surveillance.snapshot import Snapshot
from yolo_msgs.msg import DetectionArray
from geometry_msgs.msg import PoseStamped

class SnapshotPublisher(Node):
    def __init__(self):
        print('Starting snapshot publisher node...')

        super().__init__(f'snapshot_publisher')

        self.sub_curr_detections = Subscriber(DetectionArray, '/yolo/tracking', qos_profile=10)
        self.sub_curr_drone_pose = Subscriber(PoseStamped, '/visual_slam/tracking/vo_pose', qos_profile=10)
        self.snapshot_pub = self.create_publisher(Snapshot, )

        '''
        ok, so i basically want to constantly have access to the latest current pose, and use that for the latest detection array
        one approach, is i have a callback that constantly updates a field with the current pose, and then just call that field whenever
            this feels too jank?
        other, is that I synchronize the DetArray and PoseStamped messages coming in
            sometimes, we may not produce any snapshots
            other times, we will
        '''

        self.ts = ApproximateTimeSynchronizer(
            [self.sub_curr_drone_pose, self.sub_curr_detections],
            queue_size=10,
            slop=1,
            allow_headerless=False
        )

        self.ts.registerCallback(self.produce_snapshots)

        self.prev_detections = []

    # # idk if this is optimal - def not - why - explain
    # def get_curr_pose(self, msg : PoseStamped):
    #     self.current_drone_pose = msg

    def produce_snapshots(self, pose : PoseStamped, curr_detections : DetectionArray):

        new_detections = self.get_new_detections(curr_detections, self.prev_detections)
        self.prev_detections = curr_detections

        # produce snapshots
        snaps = self.produce_snapshot(pose, new_detections)

    def get_new_detections(self, curr_detections : DetectionArray, prev_detections : DetectionArray) -> DetectionArray:
        # [1, 2, 3, 4]
        # [2, 3, 4, 5]

        curr_det_ids = [det.class_id for det in curr_detections.detections]
        prev_det_ids = [det.class_id for det in prev_detections.detections]

        # considers only the new detections, not old ones leaving the scene
        new_det_ids = list(set(curr_det_ids) - set(prev_det_ids))
        new_detections = []
        for det in curr_detections.detections:
            if det.class_id in new_det_ids:
                new_detections.append(det)
        
        return new_detections

    def produce_snapshot(self, pose : PoseStamped, detections : DetectionArray) -> List[Snapshot]:
        # dubious
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SnapshotPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()