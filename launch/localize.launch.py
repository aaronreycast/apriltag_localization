import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('apriltag_localization')
    default_map = os.path.join(pkg_share, 'config', 'tag_map.yaml')

    tag_map_path = LaunchConfiguration('tag_map_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'tag_map_path',
            default_value=default_map,
            description='Absolute path to tag_map.yaml',
        ),
        Node(
            package='apriltag_localization',
            executable='vessel_localizer',
            name='vessel_localizer',
            output='screen',
            parameters=[{'tag_map_path': tag_map_path}],
        ),
    ])
