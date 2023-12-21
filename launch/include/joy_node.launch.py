from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    # Launch Arguments
    dev = LaunchConfiguration('dev', default='/dev/input/js0')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
        parameters=[
            {'dev': dev}
        ]
    )

    return LaunchDescription([
        joy_node
    ])
