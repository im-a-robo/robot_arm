from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    # Launch Arguments

    control = Node(
        package='control',
        executable='control',
        name='control',
        output='screen',
    )

    return LaunchDescription([
        control
    ])