from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
            # # Launch the MoveIt2 config
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource([
            #         PathJoinSubstitution([
            #             FindPackageShare('moveit2_dual_arm'),
            #             'launch',
            #             'demo.launch.py'
            #         ])
            #     ])
            # ),

        # Run hand_gesture_capture
        Node(
            package='hand_gesture_capture',
            executable='hand_publisher_node',
            name='hand_publisher_node',
            output='screen'
        ),

        # Run control_robot: move_to_pose
        Node(
            package='control_robot',
            executable='move_to_pose',
            name='move_to_pose',
            output='screen'
        ),

        # Run control_robot: move_to_pose_r1
        Node(
            package='control_robot',
            executable='move_to_pose_r1',
            name='move_to_pose_r1',
            output='screen'
        ),
    ])
