from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    Shutdown,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import rclpy
from rclpy.node import Node as RclpyNode
from launch.substitutions import TextSubstitution
from launch.actions import OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource


def launch_image_to_jpeg_nodes(context, *args, **kwargs):
    fused_topic = LaunchConfiguration("fused_image_topic").perform(context)
    yolo_dbg_topic = LaunchConfiguration("yolo_debug_image_topic").perform(context)

    image_to_jpeg_1 = ExecuteProcess(
        cmd=[
            "ros2", "run", "utils", "image_to_jpeg_node",
            "--ros-args", "-p",
            f"topic:={fused_topic}"
        ],
        output="screen",
    )

    image_to_jpeg_2 = ExecuteProcess(
        cmd=[
            "ros2", "run", "utils", "image_to_jpeg_node",
            "--ros-args", "-p",
            f"topic:={yolo_dbg_topic}"
        ],
        output="screen",
    )

    return [image_to_jpeg_1, image_to_jpeg_2]

def generate_launch_description():
    dev_nums_arg = DeclareLaunchArgument("dev_nums", default_value="[0, 2]")
    input_image_topic_arg = DeclareLaunchArgument("input_image_topic", default_value="/fused/image_raw")
    use_debug_arg = DeclareLaunchArgument("use_debug", default_value="1")
    fused_image_topic_arg = DeclareLaunchArgument("fused_image_topic", default_value="/fused/image_raw")
    yolo_debug_image_topic_arg = DeclareLaunchArgument("yolo_debug_image_topic", default_value="/yolo/dbg_image")

    dev_nums = LaunchConfiguration("dev_nums")
    input_image_topic = LaunchConfiguration("input_image_topic")
    use_debug = LaunchConfiguration("use_debug")
    fused_image_topic = LaunchConfiguration("fused_image_topic")
    yolo_debug_image_topic = LaunchConfiguration("yolo_debug_image_topic")

    fisheye_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("mmlove_GS720P02_camera"),
                "launch",
                "multiple_fisheye.launch.py",
            ])
        ]),
        launch_arguments={"dev_nums": dev_nums}.items(),
    )

    fisheye_fuser = ExecuteProcess(
        cmd=["ros2", "run", "surveillance", "multi_camera_fuser_node"],
        output="screen"
    )

    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("yolo_bringup"),
                "launch",
                "yolo.launch.py"
            ])
        ]),
        launch_arguments={
            "input_image_topic": input_image_topic,
            "use_debug": use_debug,
        }.items(),
    )

    snapshot_maker = ExecuteProcess(
        cmd=["ros2", "run", "surveillance", "snapshot_publisher_node"],
        output="screen"
    )

    
    rosbridge_server = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rosbridge_server"),
                "launch",
                "rosbridge_websocket_launch.xml",
            ])
        )
    )

    return LaunchDescription([
        dev_nums_arg,
        input_image_topic_arg,
        use_debug_arg,
        fused_image_topic_arg,
        yolo_debug_image_topic_arg,
        
        fisheye_launch,
        fisheye_fuser,
        yolo_launch,
        snapshot_maker,
        OpaqueFunction(function=launch_image_to_jpeg_nodes),
        rosbridge_server
    ])


# def check_visual_slam_topics(context, *args, **kwargs):
#     rclpy.init()
#     node = RclpyNode("visual_slam_topic_checker")
#     topic_names = [t[0] for t in node.get_topic_names_and_types()]
#     node.destroy_node()
#     rclpy.shutdown()

#     visual_slam_topics = [t for t in topic_names if t.startswith("/visual_slam")]
#     realsense_topics = [t for t in topic_names if t.startswith("/camera")]
#     if not visual_slam_topics or not realsense_topics:
#         print("Error: need both visual_slam and realsense topics to be up.")
#         return [Shutdown(reason="Missing /visual_slam or /camera topics")]
#     else:
#         print("/visual_slam topics found:", visual_slam_topics)
#         return []
