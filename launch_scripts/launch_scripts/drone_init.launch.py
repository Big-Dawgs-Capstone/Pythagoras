from typing import List
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import ast


def generate_launch_description():
    """
    Launch surveillance drone with object detection, VSLAM, and autonomous navigation capabilities and edge-device communication.
    """

    DeclareLaunchArgument(
        'dev_nums',
        default_value='[-1,-1,-1,-1]',
        description='Array to describe location of each fisheye camera...'
    ),

    return LaunchDescription([])
