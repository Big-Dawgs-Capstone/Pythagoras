from typing import List
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import ast

# two steps for ROS 2 to launch nodes through launch file
# 1) building the LaunchDescription object
# 2) executing the actions within it
#   ("launch context" available right now aka args passed to launch file, etc)
#   nesting a function within OpaqueFunction gives the function "launch context" 
#   use a OpaqueFunction when writing normal python code ***

def launch_fisheyes(context, *args, **kwargs):
    try:
        dev_nums : List[int] = ast.literal_eval(
            LaunchConfiguration('dev_nums').perform(context)
        )
        # check if List[int] actually produced
        assert isinstance(dev_nums, list)
        # return true if all items in array are true
        # don't like the is_instance check ngl
        assert all((isinstance(i, int) and i > -1) for i in dev_nums)
    # don't like generic Exception catch --> could be covering up for other bugs
    except Exception as e:
        raise RuntimeError(f'Invalid array of device numbers...')

    if dev_nums is [-1, -1, -1, -1]:
        raise RuntimeError(f'Array of device numbers must be provided...')

    # generate list of launch actions
    actions = []
    for i in range(len(dev_nums)):
        actions.append(
            GroupAction([
                PushRosNamespace(f'camera_{i}'),
                Node(
                    package='mmlove_GS720P02_camera',
                    executable='fisheye_camera_publisher',
                    name=f'fisheye_camera_{i}_publisher',
                    output='screen',
                    parameters=[
                        {'dev_num': dev_nums[i]},
                    ]
                )
            ])
        )

    return [GroupAction([
        PushRosNamespace(f'fisheye'),
        *actions
    ])]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'dev_nums',
            default_value='[-1,-1,-1,-1]',
            description='Array to describe location of each fisheye camera...'
        ),
        OpaqueFunction(function=launch_fisheyes)
    ])
