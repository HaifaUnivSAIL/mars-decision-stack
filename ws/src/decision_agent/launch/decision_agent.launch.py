from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('decision_agent')
    default_policy = os.path.join(package_share, 'config', 'policy.yaml')
    default_mission = os.path.join(package_share, 'config', 'mission.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('policy_config', default_value=default_policy),
        DeclareLaunchArgument('mission_config', default_value=default_mission),
        Node(
            package='decision_agent',
            executable='policy_node',
            name='decision_agent',
            output='screen',
            parameters=[
                LaunchConfiguration('policy_config'),
                LaunchConfiguration('mission_config'),
            ],
        ),
    ])
