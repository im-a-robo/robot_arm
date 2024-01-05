import os

from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro

def generate_launch_description():
    # ROS Packages
    robot_arm = get_package_share_directory('robot_arm')
    
    # Launch Arguments
    dev = LaunchConfiguration('dev', default='/dev/input/js0')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher', default='false')

    # URDF
    urdf_dir = os.path.join(robot_arm, 'description')
    urdf_file = os.path.join(urdf_dir, 'robot_arm.urdf.xacro')
    # with open(urdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    robot_desc = xacro.process_file(urdf_file).toxml()

    # Nodes
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_arm, 'launch', 'include/control.launch.py')
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

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_arm, 'launch', 'include/rviz.launch.py')
        )
    )

    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 name='joint_state_publisher',
                                 output='screen',
                                 condition = IfCondition(use_joint_state_publisher),
                                 )
    joint_state_publisher_gui = Node(package='joint_state_publisher_gui',
                                    executable='joint_state_publisher_gui',
                                    name='joint_state_publisher_gui',
                                    output='screen',
                                    condition = UnlessCondition(use_joint_state_publisher),
                                    )

    robot_state_publisher = Node(package='robot_state_publisher',
                                executable='robot_state_publisher',
                                name='robot_state_publisher',
                                output='screen',
                                parameters=[{
                                    'use_sim_time': use_sim_time,
                                    'robot_description': robot_desc
                                }])
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'robot_arm'],
                    output='screen')


    return LaunchDescription([
        DeclareLaunchArgument(
            'dev',
            default_value='/dev/input/js0',
            description='Joystick device'
        ),

        control,
        joy_node,
        rviz,
        # joint_state_publisher,
        robot_state_publisher,
        # joint_state_publisher_gui,
        gazebo,
        spawn_entity,
    ])