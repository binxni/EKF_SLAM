from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = {
        'horizon_sec': 1.6,
        'dt': 0.1,
        'forward_only': False,
        'normalize': False,
        'downsample': 1,
        'output_csv_file': 'data_log.csv'
    }

    return LaunchDescription([
        Node(
            package='ekf_slam',
            executable='global_to_polar_node',
            name='global_to_polar',
            remappings=[('/pf/pose/odom', '/pf/pose/odom'),
                        ('/planned_path_with_velocity', '/planned_path_with_velocity')]
        ),
        Node(
            package='ekf_slam',
            executable='data_logger_node',
            name='data_logger',
            remappings=[('/planned_path_with_velocity', '/planned_path_with_velocity')],
            parameters=[params]
        )
    ])
