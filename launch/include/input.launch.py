from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    # Launch Arguments

    input = Node(
        package='input',
        executable='input',
        name='input',
        output='screen',
    )

    return LaunchDescription([
        input
    ])