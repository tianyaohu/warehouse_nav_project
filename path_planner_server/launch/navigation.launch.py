import os
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery.yaml')
    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')

    filters_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'filter.yaml')

    rviz_config_dir = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'pathplanning.rviz')
    

    return LaunchDescription([     
        DeclareLaunchArgument(
            'map_file',
            default_value="warehouse_map_sim.yaml", 
            description='name of map_file within map_server/config'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value="True", 
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
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_base_link_2_robot_base_link',
            output='screen',
            emulate_tty=True,
            arguments=['0', '0', '0', '0', '0', '0', 'robot_base_link', 'base_link']
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}, 
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
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml],
            remappings=[('/cmd_vel', '/robot/cmd_vel')]
        ),


        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]
        ),
            
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            parameters=[recovery_yaml],
            output='screen'
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml],
            remappings=[('/cmd_vel', '/robot/cmd_vel')]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml]),

        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")},
                        {'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'controller_server',
                                        'planner_server',
                                        'recoveries_server',
                                        'bt_navigator',
                                        'filter_mask_server',
                                        'costmap_filter_info_server',
                                        ]}])
    ])