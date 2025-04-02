from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time

@dataclass
class Snapshot:
    detection_time : Time
    class_object : str
    conf_object : float
    image_object : Image
    drone_pose : PoseStamped