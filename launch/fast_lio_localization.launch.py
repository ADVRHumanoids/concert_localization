"""FAST-LIO localization and base-frame odometry for Concert; no base estimation."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('concert_localization')
    perception_share = get_package_share_directory('perception_utils_ros2')
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    args = [
        DeclareLaunchArgument('map_dir', description='Optimized map directory containing map.pcd, poses.csv and keyframes'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('imu_topic', default_value='/xbotcore/imu/vectornav_imu_link'),
        DeclareLaunchArgument('fast_lio_config', default_value=os.path.join(share, 'config', 'fast_lio_localization.yaml')),
        DeclareLaunchArgument('bridge_config', default_value=os.path.join(share, 'config', 'fast_lio_odom_bridge.yaml')),
        DeclareLaunchArgument('cloud_destination_topic', default_value='/merged_cloud'),
    ]

    fast_lio = Node(
        package='fast_lio', executable='fastlio_mapping',
        namespace='localization', name='laser_mapping', output='screen',
        parameters=[LaunchConfiguration('fast_lio_config'), {
            'use_sim_time': use_sim_time,
            'mode': 'localization',
            'common.imu_topic': LaunchConfiguration('imu_topic'),
            'localization.map_dir': LaunchConfiguration('map_dir'),
            'localization.update_map': False,
        }],
    )

    bridge = Node(
        package='lio_mapping', executable='fast_lio_odom_bridge',
        namespace='localization', name='fast_lio_odom_bridge', output='screen',
        parameters=[LaunchConfiguration('bridge_config'), {'use_sim_time': use_sim_time}],
        remappings=[('~/odometry', '/localization/base_odometry')],
    )

    map_pcd = GroupAction(scoped=True, actions=[
        SetParameter(name='use_sim_time', value=use_sim_time),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(perception_share, 'launch', 'pcd_to_pointcloud.launch.py')),
            launch_arguments={
                'pcd_file': PathJoinSubstitution([LaunchConfiguration('map_dir'), 'map.pcd']),
                'source_frame': 'map', 'base_link_frame': 'map',
                'cloud_topic': '/map_pcd', 'publishing_period_ms': '10000',
            }.items(),
        ),
    ])

    perception_share_dir = get_package_share_directory('perception_utils_ros2')
    merge_pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share_dir, 'launch', 'cloud_multi_merger.launch.py'),
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'output_topic': LaunchConfiguration('cloud_destination_topic')
                         }.items()
    )
 

    return LaunchDescription(args + [merge_pointcloud_launch, fast_lio, bridge, map_pcd])
