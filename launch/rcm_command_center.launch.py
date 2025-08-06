from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # RCM Manager
        Node(
            package='da_vinci_tool_integration',
            executable='rcm_manager',
            name='rcm_manager',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        ),

        # GUI Pose Publisher (RCM pose control)
        Node(
            package='da_vinci_tool_integration',
            executable='gui_pose_publisher',
            name='gui_pose_publisher',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        ),

        # PS5 Joystick driver
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[
                {'device_id': 0},
                {'deadzone': 0.05},
                {'autorepeat_rate': 20.0},
            ]
        ),

        # PS5 joystick teleop to RCM nudge commands
        Node(
            package='da_vinci_tool_integration',
            executable='ps5_teleop_rcm',
            name='ps5_teleop_rcm',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        )
    ]) 