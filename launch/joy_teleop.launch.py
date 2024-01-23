from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    joy_config = LaunchConfiguration('joy_config')
    config_filepath = LaunchConfiguration('config_filepath')

    return LaunchDescription([
        DeclareLaunchArgument('joy_config', default_value='default.yaml'),
        DeclareLaunchArgument('config_filepath', default_value=[
            TextSubstitution(text=os.path.join(
                get_package_share_directory('turtlebot3_full_joy'), 'config', '')), joy_config]),
        Node(
            package='joy',
            executable='joy_node',
            output='log',
        ),
        Node(
            package='turtlebot3_full_joy',
            executable='joy_manager',
            output='screen',
            parameters=[config_filepath],
        ),
    ])
