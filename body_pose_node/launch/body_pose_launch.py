
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='body_pose_node',
            executable='body_pose_transformer',
            name='body_pose_transformer',
            output='screen'
        )
    ])

