from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")


    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=["--x", "0", "--y", "0", "--z", "0.100", "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                "--frame-id", "base_footprint_ekf", "--child-frame-id", "imu_link_ekf"]
    )
    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("concert_localization"), "config", "ekf.yaml"),
                    {'use_sim_time': use_sim_time}]
    )

    imu_republisher_cpp = Node(
        package="concert_localization",
        executable="imu_republisher",
        parameters=[{'use_sim_time': use_sim_time}]
    )
    return LaunchDescription([
        use_sim_time_arg,
        static_transform_publisher,
        robot_localization,
        imu_republisher_cpp
    ])