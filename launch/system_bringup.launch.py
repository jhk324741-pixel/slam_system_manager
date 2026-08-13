from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    package_share = get_package_share_directory('slam_system_manager')
    default_config = os.path.join(package_share, 'config', 'system.yaml')
    default_process_config = os.path.join(package_share, 'config', 'process.yaml')
    default_sensor_config = os.path.join(package_share, 'config', 'sensor.yaml')
    default_localization_quality_config = os.path.join(
        package_share, 'config', 'localization_quality.yaml')

    config_file = LaunchConfiguration('config_file')
    process_config_file = LaunchConfiguration('process_config_file')
    sensor_config_file = LaunchConfiguration('sensor_config_file')
    localization_quality_config_file = LaunchConfiguration(
        'localization_quality_config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Absolute path to slam_system_manager system.yaml',
        ),
        DeclareLaunchArgument(
            'process_config_file',
            default_value=default_process_config,
            description='Absolute path to slam_system_manager process.yaml',
        ),
        DeclareLaunchArgument(
            'sensor_config_file',
            default_value=default_sensor_config,
            description='Absolute path to slam_system_manager sensor.yaml',
        ),
        DeclareLaunchArgument(
            'localization_quality_config_file',
            default_value=default_localization_quality_config,
            description='Absolute path to localization_quality.yaml',
        ),
        Node(
            package='slam_system_manager',
            executable='system_manager_node',
            name='system_manager',
            output='screen',
            parameters=[{
                'config_file': config_file,
                'process_config_file': process_config_file,
                'sensor_config_file': sensor_config_file,
                'localization_quality_config_file': localization_quality_config_file,
            }],
        ),
    ])
