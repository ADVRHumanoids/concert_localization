"""
Launch file to run RTAB-Map with the Gazebo simulation. This is used for testing and development of the mapping pipeline.
"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node

import os


def generate_launch_description():

    # 0. Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (useful for Gazebo simulations)'
    )

    deskewing_arg = DeclareLaunchArgument(
        'deskewing',
        default_value='false',
        description='Enable LiDAR deskewing'
    )

    rtabmap_viz_node_arg = DeclareLaunchArgument(
        'rtabmap_viz',
        default_value='true',
        description='Whether to launch the RTAB-Map visualization node'
    )

    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Whether to run RTAB-Map in localization mode (no new map creation)'
    )

    pcd_file_arg = DeclareLaunchArgument(
        'pcd_file',
        default_value="maps/pointclouds_",
        description='Prefix for the saved PCD file when the node shuts down'
    )


    ##################################################################
    #######  lAUNCH FILES  #######
    ##################################################################

    # 1. Share directories
    odom_share_dir = get_package_share_directory('concert_odometry_ros2')
    perception_share_dir = get_package_share_directory('perception_utils_ros2')
    navigation_share_dir = get_package_share_directory('concert_navigation')

    
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

    # 4. Odometry node using ICP scan matching
    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_link_projected',
            'odom_frame_id': 'odom_icp',
            'guess_frame_id': 'odom',
            'wait_for_transform': 0.5,
            'tf_tolerance': 0.1,
            'expected_update_rate': 0.0,
            # 'qos':2, # QoS 2 means 'best effort'
            'max_update_rate': 15.0,
            'deskewing': LaunchConfiguration('deskewing'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # RTAB-Map's internal parameters are strings:
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '10',
            'Icp/VoxelSize': '0.1',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0',
            'Icp/MaxTranslation': '2',
            'Icp/MaxCorrespondenceDistance': '1',
            'Icp/Strategy': '1',
            'Icp/OutlierRatio': '0.7',
            'Icp/CorrespondenceRatio': '0.01',
            'Odom/ScanKeyFrameThr': '0.4',
            'OdomF2M/ScanSubtractRadius': '0.1',
            'OdomF2M/ScanMaxSize': '15000',
            'OdomF2M/BundleAdjustment': 'false'
        }],
        remappings=[
            ('scan_cloud', '/merged_cloud')
        ],
        arguments=['-ros-args', '--log-level', 'DEBUG']
    )

    # 5. RTAB-Map SLAM/localization node
    rtabmap_node_slam = Node(
        condition=UnlessCondition(LaunchConfiguration('localization')),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link_projected',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': True,
            'approx_sync': False,
            'wait_for_transform': 0.2,
            'max_update_rate': 15.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # RTAB-Map's internal parameters
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/CreateOccupancyGrid': 'false',
            'Mem/NotLinkedNodesKept': 'false',
            'Mem/STMSize': '30',
            'Mem/LaserScanNormalK': '20',
            'Reg/Strategy': '1',
            'Icp/VoxelSize': '0.1',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0',
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '10',
            'Icp/Epsilon': '0.001',
            'Icp/MaxTranslation': '3',
            'Icp/MaxCorrespondenceDistance': '1',
            'Icp/Strategy': '1',
            'Icp/OutlierRatio': '0.7',
            'Icp/CorrespondenceRatio': '0.2'
        }],
        remappings=[
            ('scan_cloud', 'odom_filtered_input_scan')
        ],
        arguments=['-d']  # Delete the previous database (~/.ros/rtabmap.db)
    )

    rtabmap_node = Node(
        condition=IfCondition(LaunchConfiguration('localization')),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link_projected',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': True,
            'approx_sync': False,
            'wait_for_transform': 0.2,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # RTAB-Map's internal parameters
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/CreateOccupancyGrid': 'false',
            'Mem/NotLinkedNodesKept': 'false',
            'Mem/STMSize': '30',
            'Mem/LaserScanNormalK': '20',
            'Mem/IncrementalMemory':'False',
            'Mem/InitWMWithAllNodes':'True',
            'Reg/Strategy': '1',
            'Icp/VoxelSize': '0.1',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0',
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '10',
            'Icp/Epsilon': '0.001',
            'Icp/MaxTranslation': '3',
            'Icp/MaxCorrespondenceDistance': '1',
            'Icp/Strategy': '1',
            'Icp/OutlierRatio': '0.7',
            'Icp/CorrespondenceRatio': '0.2'
        }],
        remappings=[
            ('scan_cloud', 'odom_filtered_input_scan')
        ]
    )

    # 6. RTAB-Map visualization node
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'frame_id': 'base_link_projected',
            'odom_frame_id': 'odom',
            'subscribe_odom_info': True,
            'subscribe_scan_cloud': True,
            'approx_sync': False,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        remappings=[
            ('scan_cloud', 'odom_filtered_input_scan')
        ],
        condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
    )

    # 7. Map saving node
    pcd_exporting = Node(
        package='perception_utils_ros2',  # Replace with the actual package name containing pointcloud_to_pcd
        executable='pointcloud_to_pcd_node',  # Name of the executable
        name='pointcloud_to_pcd',
        parameters=[{
            'prefix': LaunchConfiguration("pcd_file"),        # Set the PCD file name prefix
            'binary': False,           # Save the PCD file in ASCII format
            'compressed': False,       # Disable compression
            'rgb': False,              # Set RGB support to false if the point cloud doesn't contain color
            'save_timer_sec': 0.0,
            'save_on_shutdown': True
        }],
        remappings=[
            ('input', '/cloud_obstacles')       # Remap the input point cloud topic to '/cloud_in'
        ],
        output='screen'
    )


    return LaunchDescription([
        use_sim_time_arg,
        deskewing_arg,
        rtabmap_viz_node_arg,
        localization_arg,
        pcd_file_arg,
        odom_launch,
        merge_pointcloud_launch,
        icp_odometry_node,
        rtabmap_node,
        rtabmap_node_slam,
        # rtabmap_viz_node,
        pcd_exporting
    ])