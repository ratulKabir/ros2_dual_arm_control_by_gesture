from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('control_robot')
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'arm_6dof_new.xacro'])
    # rviz_path = PathJoinSubstitution([pkg_share, 'urdf', 'simple_2dof.rviz'])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            # parameters=[{
            #     'robot_description': Command([FindExecutable(name='cat'), ' ', urdf_path])
            # }]
            parameters=[{
                'robot_description': Command([FindExecutable(name='xacro'), ' ', urdf_path])
            }]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # arguments=['-d', rviz_path],
        ),
    ])
