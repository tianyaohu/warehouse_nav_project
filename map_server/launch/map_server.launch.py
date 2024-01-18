import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    rviz_config_dir = os.path.join(get_package_share_directory('map_server'), 'config', 'launch_part.rviz')

    #everything default as sim
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value="False", 
            description='Turn on/off sim time setting'
        ),

        DeclareLaunchArgument(
            'map_file',
            default_value="warehouse_map_real.yaml", 
            description='name of map_file within map_server/config'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
            arguments=['-d', rviz_config_dir]),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}, 
                        {'yaml_filename':PathJoinSubstitution([FindPackageShare("map_server"), 'config', LaunchConfiguration("map_file")])}
                        ]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")},
                        {'autostart': True},
                        {'node_names': ['map_server']}]),

        ])