from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Interactive marker for setting RCM
        Node(
            package='da_vinci_tool_integration',
            executable='interactive_target_marker',
            name='interactive_target_marker',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        )
    ]) 