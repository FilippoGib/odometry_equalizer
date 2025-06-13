from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():

    config_node = os.path.join(
        get_package_share_directory('odom_equalizer'),
        'config',
        'odom_equalizer.yaml'
        )

    node=Node(
            package='odom_equalizer',
            name='odom_equalizer',
            executable='odom_equalizer_node',
            parameters=[config_node]
        )

    return LaunchDescription(
        [           
            node
        ]
    )