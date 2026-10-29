## IMU visualization launch file
# Starts imu_viz and RViz2. Pass rviz:=false to start imu_viz only.

from pathlib import Path

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    base = Path(get_package_share_directory('holoocean_viz'))
    params_file = base / 'config' / 'imu_viz.yaml'
    rviz_config = base / 'rviz' / 'imu_viz.rviz'

    with_rviz = LaunchConfiguration('rviz')

    imu_viz_node = launch_ros.actions.Node(
        name='imu_viz',
        package='holoocean_viz',
        executable='imu_viz',
        output='screen',
        parameters=[params_file],
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_config)],
        condition=IfCondition(with_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2'),
        imu_viz_node,
        rviz_node,
    ])
