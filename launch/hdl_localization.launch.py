"""
Launch file to run HDL localiization from a stored pcdc pointcloud map.
"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import (
        DeclareLaunchArgument,
        IncludeLaunchDescription,
        TimerAction,
        ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


import os


def generate_launch_description():

    ##################################################################
    #######  lAUNCH ARGUMENTS  #######
    ##################################################################
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (useful for Gazebo simulations)'
    )

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='/home/user/data/forest_ws/maps/nav_map.pcd',
        description='Path to the PCD file containing the pointcloud map for localization'
    )

    auto_localize_arg = DeclareLaunchArgument(
        'auto_localize',
        default_value='true',
        description='Whether to automatically call the global localization service after a delay'
    )

    ##################################################################
    #######  lAUNCH FILES  #######
    ##################################################################

    # 1. Share directories
    odom_share_dir = get_package_share_directory('concert_odometry_ros2')
    perception_share_dir = get_package_share_directory('perception_utils_ros2')

    
    # 2. Launch base estimation odometry
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(odom_share_dir, 'launch', 'concert_odometry.launch.py'),
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # 3. Merge Lidar point clouds
    merge_pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share_dir, 'launch', 'cloud_multi_merger.launch.py'),
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # 4. Load map as pointcloud to visualize in RViz
    map_pcd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share_dir, 'launch', 'pcd_to_pointcloud.launch.py'),
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'pcd_file': LaunchConfiguration('map_file'),
            'base_link_frame': 'map',
            'publishing_period_ms': '10000'
        }.items()
    )

    # 5. Launch HDL localization
    hdl_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('hdl_localization'), 'launch/hdl_localization.launch.py')),
        launch_arguments={
            'points_topic': '/merged_cloud',  # PointCloud2 Topic to be used for localization
            'globalmap_pcd': LaunchConfiguration('map_file'),  # Pass the map file argument to the HDL localization launch
            'use_global_localization': 'true',  # Enable global localization mode
            'plot_estimation_errors': 'false'  # Enable plotting of estimation errors
        }.items()
    )

    # 6. Change global localization engine
    global_localization_engine = TimerAction(
        condition=IfCondition(LaunchConfiguration('auto_localize')),
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/hdl_global_localization/set_engine",
                    "hdl_global_localization/srv/SetGlobalLocalizationEngine",
                    "{engine_name: {data: 'FPFH_RANSAC'}}",
                ],
                output="screen",
            )
        ],
    )
    
    # 7. Launch global localization request to service
    global_localization_request = TimerAction(
        condition=IfCondition(LaunchConfiguration('auto_localize')),
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/relocalize",
                    "std_srvs/srv/Empty",
                    "{}",
                ],
                output="screen",
            )
        ],
    )
    

    return LaunchDescription(
        [
            use_sim_time_arg,
            map_file_arg,
            auto_localize_arg,
            odom_launch,
            merge_pointcloud_launch,
            map_pcd_launch,
            hdl_localization_launch,
            global_localization_engine,
            global_localization_request
        ]
    )
    