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
    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value="warehouse_map_sim.yaml", 
            description='name of map_file within map_server/config'
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':PathJoinSubstitution([FindPackageShare("map_server"), 'config', LaunchConfiguration("map_file")])}]
        ),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        ),


    ])