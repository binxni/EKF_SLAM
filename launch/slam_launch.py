from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare('ekf_slam')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file', #lanch 인자의 이름
        default_value=PathJoinSubstitution([pkg_share, 'config', 'slam_params.yaml']),  #이 인자의 기본값
        description='Path to SLAM configuration file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_share, 'rviz', 'slam_visualization.rviz']),
        description='Path to RViz configuration file'
    )
    
    # Nodes
    slam_node = Node(
        package='ekf_slam',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
            ('map', '/map'),
            ('slam_pose', '/slam_pose')
        ]
    )
    
    lidar_processor_node = Node(
        package='lidar_ekf_slam',
        executable='lidar_processor_node',
        name='lidar_processor_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        slam_node,
    ])

