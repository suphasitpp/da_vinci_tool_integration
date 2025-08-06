from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([

        # MoveIt demo (robot model + planning)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('moveit_config'),
                    'launch',
                    'demo.launch.py'
                ])
            ])
        ),

        # IK Solver node
        Node(
            package='da_vinci_tool_integration',
            executable='ik_solver',
            name='ik_solver',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        )
    ]) 