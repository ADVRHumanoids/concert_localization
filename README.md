# concert_localization

Localization and mapping-support launch files for the Concert robot: PCD-to-Nav2-map
conversion, RTAB-Map/HDL-based global localization, AMCL-based localization, and a
local EKF-based state estimator.

## Launch files

### `pcd_to_nav2_map.launch.py`

Converts a stored PCD pointcloud map into a Nav2 occupancy grid via Octomap:

```
PCD file -> PointCloud2 /cloud_pcd -> Octomap -> /projected_map -> Nav2 map_saver_cli
```

```bash
ros2 launch concert_localization pcd_to_nav2_map.launch.py \
  map_file:=/home/user/data/forest_ws/maps/nav_map.pcd \
  output_name:=nav_map \
  min_z:=0.10 \
  max_z:=1.50 \
  resolution:=0.05 \
  save_cropped_pcd:=true
```

| Argument | Default | Description |
|---|---|---|
| `map_file` | *(required)* | Path to the input PCD file |
| `output_name` | PCD filename stem | Output folder name under `./maps/` |
| `min_z` / `max_z` | `-0.50` / `1.50` | Height band [m] kept when building the Octomap |
| `resolution` | `0.05` | Octomap / occupancy grid resolution [m] |
| `save_delay` | `5.0` | Seconds to wait before saving, to let the Octomap accumulate |
| `save_cropped_pcd` | `false` | If true, also save the height-filtered Octomap pointcloud (`/octomap_point_cloud_centers`) as a new `.pcd` file |
| `cropped_pcd_prefix` | `<output_dir>/<output_name>_cropped_` | Prefix for the cropped PCD output, only used when `save_cropped_pcd` is true |

Output:
```
./maps/<output_name>/<output_name>.yaml
./maps/<output_name>/<output_name>.pgm
./maps/<output_name>/<output_name>_cropped_<timestamp>.pcd   (if save_cropped_pcd:=true)
```

Note: the cropped PCD is derived from the Octomap's occupied-voxel centers, so it is
voxelized at `resolution` and reflects occupied cells rather than a lossless crop of
the original input points.

### `hdl_localization.launch.py`

Runs HDL-based global localization against a stored PCD pointcloud map.

| Argument | Description |
|---|---|
| `use_sim_time` | Use simulation time (useful for Gazebo) |
| `map_file` | Path to the PCD pointcloud map used for localization |
| `auto_localize` | Whether to automatically call the global localization service after a delay |

### `rtabmap.launch.py`

Lidar-only SLAM/localization pipeline built on RTAB-Map, used for testing and
development of the mapping pipeline (typically against the Gazebo simulation).
It chains together base odometry, lidar-scan merging, ICP scan-matching odometry,
RTAB-Map itself, and a final export of the obstacle cloud to a `.pcd` file:

```
concert_odometry.launch.py  ->  base odometry (odom frame)
cloud_multi_merger.launch.py -> merges lidar scans into /merged_cloud
icp_odometry (rtabmap_odom)  -> ICP scan-matching odometry -> odom_icp frame
rtabmap (rtabmap_slam)       -> SLAM (default) or localization mode, subscribes
                                 to odom_filtered_input_scan
pointcloud_to_pcd_node       -> saves /cloud_obstacles to <pcd_file><timestamp>.pcd
                                 on shutdown
```

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Use simulation time (useful for Gazebo) |
| `deskewing` | `false` | Enable LiDAR deskewing in `icp_odometry` |
| `rtabmap_viz` | `true` | Intended to toggle the `rtabmap_viz` visualization node, but that node is currently commented out of the returned `LaunchDescription` — this arg has no effect right now |
| `localization` | `false` | If `true`, runs RTAB-Map in localization mode against the existing map (no new map creation, no `-d` database wipe); if `false` (default), runs full SLAM and **deletes the previous map database** (`~/.ros/rtabmap.db`) on startup via the `-d` argument |
| `pcd_file` | `maps/pointclouds_` | Prefix for the accumulated `/cloud_obstacles` cloud, written to `<prefix><timestamp>.pcd` when the node shuts down |

Notes:
- `localization:=false` (the default) always wipes the previous RTAB-Map database —
  pass `localization:=true` to localize against an existing map without rebuilding it.
- The exported `.pcd` is only written on node shutdown (`save_timer_sec: 0.0`), so the
  launch must be stopped (e.g. `Ctrl+C`) for the file to be saved.

### `localization.launch.py`

AMCL-based localization against a saved Nav2 map: starts `nav2_map_server`, `nav2_amcl`
(configured via `config/amcl_config.yaml`), and the associated lifecycle manager.

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use simulation time |
| `map_file` | `concert_mapping/maps/robot_area.yaml` | Map yaml loaded by `map_server` |
| `amcl_config` | `config/amcl_config.yaml` | AMCL parameters |

### `local_localization.launch.py`

Local state estimation: a static transform (`base_footprint_ekf` -> `imu_link_ekf`),
`robot_localization`'s EKF node (configured via `config/ekf.yaml`), and an IMU
republisher.

## Config

- `config/amcl_config.yaml` — AMCL parameters used by `localization.launch.py`
- `config/ekf.yaml` — EKF parameters used by `local_localization.launch.py`
