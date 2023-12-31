import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # ROS packages
    robot_arm = get_package_share_directory(
        'robot_arm')

    # launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Nodes
    rviz = Node(package='rviz2',
                executable='rviz2',
                arguments=[
                    '-d', os.path.join(robot_arm, 'config',
                                 'robot_arm.rviz')

                ],
                parameters=[{
                    'use_sim_time': use_sim_time
                }]
                )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation time if true'),
        DeclareLaunchArgument('-d',
                              default_value=os.path.join(robot_arm, 'config',
                                                         'robot_arm.rviz'),
                              description='Full path to the ROS2 .rviz file to use'),

        # Node
        rviz,
    ])
