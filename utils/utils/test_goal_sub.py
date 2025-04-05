import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import cv2
from std_msgs.msg import Bool


class TestGoalSub(Node):

    def __init__(self):
        super().__init__(f'goal_sub_node')
        self.create_subscription(Bool, '/cancel', self.listener_callback, qos_profile=10)

    def listener_callback(self, msg):
        print(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestGoalSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()