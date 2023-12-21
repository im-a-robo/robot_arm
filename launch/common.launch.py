import os

from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ROS Packages
    robot_arm = get_package_share_directory('robot_arm')
    
    # Launch Arguments
    dev = LaunchConfiguration('dev', default='/dev/input/js0')

    # Nodes
    input = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_arm, 'launch', 'include/input.launch.py')
        )
    )

    joy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_arm, 'launch', 'include/joy_node.launch.py')
        ),
        launch_arguments={
            'dev': dev
        }.items()
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'dev',
            default_value='/dev/input/js0',
            description='Joystick device'
        ),

        input,
        joy_node
    ])
