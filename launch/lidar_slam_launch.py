## LiDAR SLAM launch file
# Starts lidar_bridge, optionally HoloOcean, a LiDAR-inertial SLAM and RViz2.
#   slam:=fast_lio | lio_sam | none   (default fast_lio)
#   with_sim:=true                    also start holoocean_main with config/scenarios/blueROV_lio.json
#   scenario:=<path>                  scenario JSON for with_sim
#   rviz:=false                       no RViz2

from pathlib import Path

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# Per-SLAM settings for lidar_bridge: where the SLAM's own world frame starts
# and which frame the ground-truth TF drives (LIO-SAM publishes base_link itself)
SLAM_ORIGIN = {
    # FAST-LIO2's 'camera_init' is the IMU frame at start-up
    'fast_lio': {'slam_origin_frame': 'camera_init', 'slam_origin_link': 'imu', 'slam_origin_rotation': True,
                 'ground_truth_frame': 'auv0/base_link'},
    # LIO-SAM's odometry frame starts at the LiDAR, gravity- and heading-aligned
    'lio_sam': {'slam_origin_frame': 'lio_odom', 'slam_origin_link': 'lidar', 'slam_origin_rotation': False,
                'ground_truth_frame': 'auv0/base_link_gt'},
    'none': {'slam_origin_frame': '', 'ground_truth_frame': 'auv0/base_link'},
}


def _launch_setup(context):
    base = Path(get_package_share_directory('holoocean_viz'))
    slam = LaunchConfiguration('slam').perform(context)
    if slam not in SLAM_ORIGIN:
        raise RuntimeError(f"slam must be one of {list(SLAM_ORIGIN)}, got '{slam}'")
    with_sim = LaunchConfiguration('with_sim').perform(context).lower() == 'true'
    with_rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    scenario = LaunchConfiguration('scenario').perform(context)

    nodes = []

    if with_sim:
        nodes.append(launch_ros.actions.Node(
            name='holoocean_node',
            package='holoocean_main',
            executable='holoocean_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'relative_path': False,
                'scenario_path': scenario,
                'show_viewport': True,
                'draw_arrow': False,
                'publish_commands': True,
            }],
        ))

    nodes.append(launch_ros.actions.Node(
        name='lidar_bridge',
        package='holoocean_viz',
        executable='lidar_bridge',
        output='screen',
        parameters=[str(base / 'config' / 'lidar_bridge.yaml'), SLAM_ORIGIN[slam]],
    ))

    if slam == 'fast_lio':
        nodes.append(launch_ros.actions.Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fastlio_mapping',
            output='screen',
            parameters=[str(base / 'config' / 'slam' / 'fast_lio_holoocean.yaml')],
        ))
    elif slam == 'lio_sam':
        lio_params = str(base / 'config' / 'slam' / 'lio_sam_holoocean.yaml')
        nodes.append(launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lio_map_to_odom',
            arguments=['--frame-id', 'lio_map', '--child-frame-id', 'lio_odom'],
            parameters=[{'use_sim_time': True}],
        ))
        for exe in ('imuPreintegration', 'imageProjection', 'featureExtraction', 'mapOptimization'):
            nodes.append(launch_ros.actions.Node(
                package='lio_sam',
                executable=f'lio_sam_{exe}',
                name=f'lio_sam_{exe}',
                output='screen',
                parameters=[lio_params],
            ))

    if with_rviz:
        nodes.append(launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(base / 'rviz' / 'lidar_slam.rviz')],
            parameters=[{'use_sim_time': True}],
        ))

    return nodes


def generate_launch_description():
    default_scenario = str(Path(get_package_share_directory('holoocean_viz'))
                           / 'config' / 'scenarios' / 'blueROV_lio.json')
    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='fast_lio',
                              description='fast_lio, lio_sam or none'),
        DeclareLaunchArgument('with_sim', default_value='false',
                              description='Also launch holoocean_main'),
        DeclareLaunchArgument('scenario', default_value=default_scenario,
                              description='Scenario JSON used when with_sim:=true'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2'),
        OpaqueFunction(function=_launch_setup),
    ])
