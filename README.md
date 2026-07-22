# EOD-AV

ROS 2 workspace for the EOD-AV sensor stack: 3x LUCID Arena cameras, a Hesai LiDAR, and a GNSS/GPS receiver, with launch files to bring each sensor up on its own or synchronized together.

## Packages

| Package | Build type | Description |
|---|---|---|
| [`arena_camera_node`](arena_camera_node) | `ament_cmake` (C++) | LUCID Vision Labs Arena SDK driver node. `three_cameras.launch.py` starts 3 cameras (`enp6s0`, `enp7s0`, `enp8s0`), each publishing `sensor_msgs/Image` on its own topic. |
| [`HesaiLidar_ROS_2.0`](HesaiLidar_ROS_2.0) (package `hesai_ros_driver`) | `ament_cmake` (C++) | Hesai LiDAR ROS 2 driver, vendored in-tree from [HesaiTechnology/HesaiLidar_ROS_2.0](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0) (includes the `HesaiLidar_SDK_2.0` source). `start.py` brings up the LiDAR node. |
| [`gps_bringup`](gps_bringup) | `ament_cmake` | Bringup for the NMEA GNSS/GPS serial driver (`nmea_navsat_driver`), targeting a SimpleRTK3B receiver. See [gps_bringup/README.md](gps_bringup/README.md) for hardware/heading details and troubleshooting. |
| [`eod_av_launch`](eod_av_launch) | `ament_python` | Top-level orchestration: includes the other packages' launch files to bring up camera+LiDAR (`camera_lidar_sync.launch.py`) or camera+LiDAR+GPS (`camera_lidar_gnss_sync.launch.py`) together. |

## Requirements
- ROS 2 sourced (tested against Jazzy)
- `colcon`, `rosdep`
- Vendor SDKs installed system-wide, outside of rosdep:
  - **LUCID Arena SDK** for `arena_camera_node` — install via LUCID's installer first; the build looks for `/etc/ld.so.conf.d/Arena_SDK.conf` and fails fast if it's missing.
- `nmea_navsat_driver` for `gps_bringup` is a normal rosdep/apt dependency (`ros-<distro>-nmea-navsat-driver`) — not vendored.

## Build

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Launching

Individually:

```bash
ros2 launch arena_camera_node three_cameras.launch.py
ros2 launch hesai_ros_driver start.py
ros2 launch gps_bringup gps.launch.py
```

Combined, via `eod_av_launch`:

```bash
ros2 launch eod_av_launch camera_lidar_sync.launch.py        # cameras + LiDAR
ros2 launch eod_av_launch camera_lidar_gnss_sync.launch.py   # cameras + LiDAR + GPS
```

## Repository layout

```
EOD-AV/
├── arena_camera_node/    # LUCID Arena camera driver (C++)
├── HesaiLidar_ROS_2.0/   # Hesai LiDAR driver, vendored from upstream
├── gps_bringup/          # NMEA GPS serial driver bringup
├── eod_av_launch/        # combined launch files for the above
└── setup/                # host-level provisioning for PTP time sync — see setup/README.md
```

## Time synchronization (PTP)

This is a dataset-creation project: recordings from the 3 cameras, the LiDAR, and the GNSS receiver only fuse correctly if every sensor timestamps its data off the **same clock**. Each device free-runs on its own clock otherwise, and their drift compounds over a recording session, corrupting alignment between camera frames, LiDAR scans, and GNSS/RTK position. IEEE 1588 PTP is what ties all of them to one shared, disciplined clock — that's what `setup/` provisions.

[`setup/`](setup/) holds the host-side PTP provisioning script (`setup_ptp_sync.sh`) — see [setup/README.md](setup/README.md) for what the script does and why each part is necessary.

## License

Per-package: `gps_bringup` and `arena_camera_node` are MIT; `hesai_ros_driver` (`HesaiLidar_ROS_2.0`) is vendored upstream under Hesai's BSD license — see each package's `LICENSE`/`package.xml` for specifics.
