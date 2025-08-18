from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # Include real hardware robot with forward_position_controller
    hardware_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lbr_bringup'),
                'launch',
                'hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'ctrl': 'forward_position_controller',
            'model': 'med7',
        }.items()
    )
    ld.add_action(hardware_robot_launch)

    # RViz for hardware robot visualization
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='mock_robot_rviz',
    #     arguments=['-d', PathJoinSubstitution([
    #         FindPackageShare('lbr_bringup'),
    #         'config',
    #         'mock.rviz'
    #     ])]
    # )
    # ld.add_action(rviz_node)

    return ld 