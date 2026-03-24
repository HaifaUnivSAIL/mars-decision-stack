from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('manual_runtime_test')
    default_config = os.path.join(package_share, 'config', 'keyboard.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        Node(
            package='manual_runtime_test',
            executable='keyboard_teleop',
            name='manual_runtime_test',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('config')],
        ),
    ])
