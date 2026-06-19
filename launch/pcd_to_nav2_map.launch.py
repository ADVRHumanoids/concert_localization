"""
Launch file to convert a PCD file into a Nav2 map using Octomap.

Pipeline:
PCD file -> PointCloud2 /cloud_pcd -> Octomap -> /projected_map -> Nav2 map_saver_cli

Example:
ros2 launch your_package pcd_to_nav2_map.launch.py \
  map_file:=/home/user/data/forest_ws/maps/nav_map.pcd \
  output_name:=nav_map \
  min_z:=0.10 \
  max_z:=1.50 \
  resolution:=0.05

Output:
./maps/<output_name>/<output_name>.yaml
./maps/<output_name>/<output_name>.pgm
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    OpaqueFunction,
    LogInfo,
    EmitEvent,
)

from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def create_output_and_save_action(context, *args, **kwargs):
    map_file = LaunchConfiguration('map_file').perform(context)
    output_name = LaunchConfiguration('output_name').perform(context)
    save_delay = LaunchConfiguration('save_delay').perform(context)

    if output_name == '':
        output_name = os.path.splitext(os.path.basename(map_file))[0]

    # Create ./maps/<output_name> relative to the directory where ros2 launch is run
    maps_root = os.path.join(os.getcwd(), 'maps')
    output_dir = os.path.join(maps_root, output_name)

    if not os.path.exists(map_file):
        return [
            LogInfo(msg=f"ERROR: Input PCD file does not exist: {map_file}"),
            EmitEvent(event=Shutdown(reason=f"Input PCD file does not exist: {map_file}")),
        ]

    os.makedirs(maps_root, exist_ok=True)

    if os.path.exists(output_dir):
        return [
            LogInfo(msg=f"ERROR: Output folder already exists: {output_dir}"),
            EmitEvent(event=Shutdown(reason=f"Output folder already exists: {output_dir}")),
        ]

    os.makedirs(output_dir)

    # map_saver_cli will create:
    #   <output_prefix>.yaml
    #   <output_prefix>.pgm
    output_prefix = os.path.join(output_dir, output_name)

    save_map = TimerAction(
        period=float(save_delay),
        actions=[
            LogInfo(
                msg=f"Saving projected octomap to Nav2 map: {output_name}.yaml / .pgm"
            ),
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                    '-f', output_prefix,
                    '--ros-args',
                    '-r', 'map:=/projected_map',
                ],
                output='screen',
            ),
        ],
    )

    return [
        save_map,
        LogInfo(msg=f"Created output folder: {output_dir}"),
    ]


def generate_launch_description():

    ##################################################################
    # Launch arguments
    ##################################################################

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        description='Path to the PCD file containing the pointcloud map'
    )

    output_arg = DeclareLaunchArgument(
        'output_name',
        default_value='',
        description='Output folder name. If empty, it uses the PCD filename stem.'
    )

    min_z_arg = DeclareLaunchArgument(
        'min_z',
        default_value='-0.50',
        description='Minimum height [m] to keep from the pointcloud.'
    )

    max_z_arg = DeclareLaunchArgument(
        'max_z',
        default_value='1.50',
        description='Maximum height [m] to keep from the pointcloud.'
    )

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution [m] of the Octomap and OccupancyGrid map.'
    )

    save_delay_arg = DeclareLaunchArgument(
        'save_delay',
        default_value='5.0',
        description='Seconds to wait before calling map_saver_cli.'
    )

    ##################################################################
    # Paths
    ##################################################################

    perception_share_dir = get_package_share_directory('perception_utils_ros2')

    ##################################################################
    # 1. Load PCD as PointCloud2 and publish it
    ##################################################################

    map_pcd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                perception_share_dir,
                'launch',
                'pcd_to_pointcloud.launch.py'
            )
        ),
        launch_arguments={
            'pcd_file': LaunchConfiguration('map_file'),
            'base_link_frame': 'map',
            'publishing_period_ms': '1000',
        }.items()
    )

    ##################################################################
    # 2. Convert PointCloud2 to Octomap and projected 2D OccupancyGrid
    ##################################################################

    pcl_to_2d_occ = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server_to_occ_grid',
        output='screen',
        parameters=[
            {'frame_id': 'map'},
            {'resolution': LaunchConfiguration('resolution')},

            # Height filter.
            {'point_cloud_min_z': LaunchConfiguration('min_z')},
            {'point_cloud_max_z': LaunchConfiguration('max_z')},
        ],
        remappings=[
            ('cloud_in', '/cloud_pcd'),
        ]
    )

    ##################################################################
    # 3. Create output folder and save /projected_map with Nav2 map_saver
    ##################################################################

    save_map_runtime_action = OpaqueFunction(
        function=create_output_and_save_action
    )

    return LaunchDescription([
        map_file_arg,
        output_arg,
        min_z_arg,
        max_z_arg,
        resolution_arg,
        save_delay_arg,

        map_pcd_launch,
        TimerAction(
            period=1.0,
            actions=[
                pcl_to_2d_occ]
        ),
        save_map_runtime_action,
    ])