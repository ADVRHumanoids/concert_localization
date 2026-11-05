import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value=PathJoinSubstitution([
            get_package_share_directory("concert_mapping"),
            "maps",
            "robot_area.yaml"
        ]),
        description="Absolute path to the map .yaml loaded by map_server. "
                    "Override with map_file:=/abs/path/to/map.yaml"
    )

    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value=os.path.join(
            get_package_share_directory("concert_localization"),
            "config",
            "amcl_config.yaml"
        )
    )


    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config = LaunchConfiguration("amcl_config")
    lifecycle_nodes = ["map_server", "amcl"]

    # Map yaml for map_server: from the map_file argument (defaults to
    # concert_mapping/maps/robot_area.yaml), overridable on the command line.
    map_path = LaunchConfiguration("map_file")


    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_path},
            {"use_sim_time" : use_sim_time}
        ]
    )

    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            amcl_config,
            {"use_sim_time" : use_sim_time}
        ]
    )

    nav2_life_cycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time" : use_sim_time},
            {"autostart": True}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_file_arg,
        amcl_config_arg,
        nav2_map_server,
        nav2_amcl,
        nav2_life_cycle_manager
    ])
