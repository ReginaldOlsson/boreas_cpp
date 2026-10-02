# Boreas to ROS 2

Convert a [Boreas](https://www.boreas.utias.utoronto.ca/) sequence on disk into a ROS 2 bag for playback in RViz, Autoware, or other tools.

The node reads lidar `.bin` frames, camera `.png` images, Applanix ground truth / GNSS / IMU, and calibration files, then writes a time-synchronized bag with `/clock`, sensor topics, optional ground truth, and a connected TF tree.

## TF tree

```
map ──(/tf, dynamic, GNSS+UTM)──► applanix ──(/tf_static)──► base_link ──(/tf_static)──► lidar ──(/tf)──► lidar_odom
                                    │               └──(/tf_static)──► camera_lidar ──(/tf)──► camera_odom
                                    └──(/tf_static)──► imu
```

- **`map → applanix`** on `/tf` — GNSS from `gps_post_process.csv` (Applanix INS frame) interpolated at lidar stamps, projected via `map_projector_info.yaml`.
- **`applanix → base_link`** on `/tf_static` — from `T_applanix_lidar.txt` and `base_link → lidar` (`T_applanix_base = T_applanix_lidar × inv(T_base_link_lidar)`).
- **`applanix → imu`** on `/tf_static` — co-located (zero translation); **Rz(-90°)** for a right-handed IMU frame (x-back, y-right, z-up). `imu_raw.csv` is x-back / y-left / z-up (left-handed). The bag converts it into that right-handed frame before publishing: acceleration Y is negated, and gyro X and Z are negated. Post-processed `imu.csv` is left unchanged and should use `imu_frame: applanix`.
- **`base_link → lidar`** on `/tf_static` — yaw-only offset (default **-45°**), zero translation.
- **`base_link → camera_lidar`** on `/tf_static` — `T_base_link_lidar × inv(T_camera_lidar)` from Boreas calibration.
- **`lidar → lidar_odom`** on `/tf` — `inv(T_enu_lidar)` from `lidar_poses.csv` at each lidar stamp.
- **`camera_lidar → camera_odom`** on `/tf` — `inv(T_enu_camera)` from `camera_poses.csv` at each camera stamp.

Static transforms are in [`config/boreas_tf.yaml`](config/boreas_tf.yaml). Set `map_projector_info_path` to your Autoware map's `map_projector_info.yaml` (LocalCartesianUTM). For other sequences without precomputed values, set `use_precomputed_static_tf: false` to recompute from `calib/T_camera_lidar.txt`.

## TF QoS (ROS 2 / `tf2_ros` convention)

Bag TF topics use the same QoS profiles as [`tf2_ros/qos.hpp`](https://github.com/ros2/geometry2/blob/ros2/tf2_ros/include/tf2_ros/qos.hpp) so RViz, `tf2_ros`, and Autoware can subscribe correctly.

### `/tf_static` — `tf2_ros::StaticBroadcasterQoS`

| Policy | Value |
|--------|-------|
| Reliability | **Reliable** |
| Durability | **Transient Local** (late joiners receive the last message) |
| History | **Keep Last** |
| Depth | **1** |

Static transforms are published once; subscribers that start later (RViz, navigation) must receive that latched state. Wrong durability (e.g. Volatile) causes disconnected trees.

### `/tf` — `tf2_ros::DynamicBroadcasterQoS`

| Policy | Value |
|--------|-------|
| Reliability | **Reliable** |
| Durability | **Volatile** (only live samples; no latch) |
| History | **Keep Last** |
| Depth | **100** |

Dynamic transforms stream continuously; `tf2` buffers recent history for interpolation. Reliable delivery is required for standard tools (`rviz2`, `tf2_monitor`).

Verify after recording:

```bash
ros2 bag info /path/to/output_bag -v   # check offered QoS on /tf and /tf_static
```

## Dataset layout

Expected directory structure (per Boreas release):

```
boreas-YYYY-MM-DD-HH-MM/
  lidar/<timestamp>.bin
  camera/<timestamp>.png
  applanix/
    lidar_poses.csv
    camera_poses.csv
    gps_post_process.csv
    imu_raw.csv          # default IMU source (or imu.csv)
  calib/
    T_camera_lidar.txt
```

## Build

```bash
cd <your_ros_ws>
colcon build --packages-select boreas
source install/setup.bash
```

## Run

```bash
ros2 launch boreas boreas.launch.xml \
  data_path:=/path/to/boreas-YYYY-MM-DD-HH-MM \
  output_bag:=/path/to/output_bag \
  map_projector_info_path:=/path/to/map/map_projector_info.yaml \
  bag_duration_sec:=300
```

Launch arguments:

| Argument | Description |
|----------|-------------|
| `data_path` | Path to extracted Boreas sequence |
| `output_bag` | Output bag directory |
| `bag_duration_sec` | Cap bag length in seconds (`0` = full sequence) |
| `map_projector_info_path` | Autoware `map_projector_info.yaml` for GNSS→map projection (required for `map→base_link`) |
| `config` | Main config (topics, intrinsics, ground truth) |
| `tf_config` | TF frames and static transforms |

## Bag contents

| Topic | Type | Notes |
|-------|------|-------|
| `/boreas/pointcloud` | `sensor_msgs/PointCloud2` | Frame: `lidar` |
| `/boreas/image/compressed` | `sensor_msgs/CompressedImage` | Frame: `camera_lidar` |
| `/boreas/camera_info` | `sensor_msgs/CameraInfo` | |
| `/tf` | `tf2_msgs/TFMessage` | `map→applanix`, `lidar→lidar_odom`, `camera_lidar→camera_odom` |
| `/tf_static` | `tf2_msgs/TFMessage` | `applanix→base_link`, `applanix→imu`, `base_link→{lidar, camera_lidar}` |
| `/map/map_projector_info` | `autoware_map_msgs/MapProjectorInfo` | Transient local; matches Autoware map |
| `/clock` | `rosgraph_msgs/Clock` | Use with `--clock` on play |
| `/boreas/ground_truth/lidar_odom` | `nav_msgs/Odometry` | Optional |
| `/boreas/ground_truth/camera_odom` | `nav_msgs/Odometry` | Optional |
| `/boreas/gnss/fix` | `sensor_msgs/NavSatFix` | Frame: `applanix`; 200 Hz |
| `/boreas/imu` | `sensor_msgs/Imu` | Frame: `imu`; from `applanix/imu_raw.csv` at 200 Hz (angvel + accel; orientation unset), converted from the left-handed raw axes into the right-handed `imu` frame. Linked via static `applanix→imu` (Rz(-90°)). For post-processed `imu.csv`, set `imu_csv: imu.csv` and `imu_frame: applanix` (no axis conversion). |

## Playback

```bash
ros2 bag play /path/to/output_bag --clock
ros2 run tf2_tools view_frames   # verify connected TF tree
ros2 topic echo /map/map_projector_info --once
ros2 bag info /path/to/output_bag -v   # QoS on /tf, /tf_static, /map/map_projector_info
```

In RViz, set **Fixed Frame** to `map` and enable **Use sim time**.

## Configuration

- [`config/boreas.yaml`](config/boreas.yaml) — bag paths, topics, camera intrinsics, ground-truth/GNSS toggles.
- [`config/boreas_tf.yaml`](config/boreas_tf.yaml) — frame names, `map_projector_info_path`, yaw offset, precomputed static transforms.

Key TF parameters:

```yaml
map_projector_info_path: "/path/to/map/map_projector_info.yaml"
base_link_to_lidar_yaw_deg: -45.0
use_precomputed_static_tf: true   # false = compute from calib/T_camera_lidar.txt
write_tf: true
write_map_projector_info: true
```

## Examples

Pointcloud projected to image:

![Image with pointcloud](doc/image_with_intensity_pc.png)

Pointcloud with RGB image:

![Image with painted pointcloud](doc/image_with_rgb_pc.png)
