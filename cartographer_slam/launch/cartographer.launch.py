import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    configuration_basename = 'cartographer.lua'

    rviz_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config', 'cartographer.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value="False", 
            description='Turn on/off sim time setting'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
            arguments=['-d', rviz_config_dir]),

        Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),

        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),

    ]) 