# EOD-AV

ROS 2 workspace for the EOD-AV sensor stack: **3x LUCID Arena cameras, a Hesai
LiDAR, a Continental ARS430 radar, and a GNSS/RTK receiver** (simpleRTK3B +
UM982 dual-antenna heading), with launch files to bring each sensor up on its
own or all of them together, plus host-side PTP time-sync provisioning.

## Packages

| Package | Build type | Description |
|---|---|---|
| [`arena_camera_node`](arena_camera_node) | `ament_cmake` (C++) | LUCID Vision Labs Arena SDK driver node. `three_cameras.launch.py` starts 3 cameras (`enp6s0`, `enp7s0`, `enp8s0`), each publishing `sensor_msgs/Image` on its own topic. |
| [`HesaiLidar_ROS_2.0`](HesaiLidar_ROS_2.0) (package `hesai_ros_driver`) | `ament_cmake` (C++) | Hesai LiDAR ROS 2 driver, vendored in-tree from [HesaiTechnology/HesaiLidar_ROS_2.0](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0) (includes the `HesaiLidar_SDK_2.0` source). `start.py` brings up the LiDAR node (and an RViz). |
| [`ars430_ros_publisher`](ars430_ros_publisher) | `ament_cmake` (C++) | Continental ARS430 radar driver (RDI protocol v2, multicast). libpcap sniffer + v2 decoder, runtime-tunable filters, PointCloud2 + marker visualizers. `radar_live.launch.py` runs the full pipeline. See [ars430_ros_publisher/README.md](ars430_ros_publisher/README.md). |
| [`gps_bringup`](gps_bringup) | `ament_cmake` | GNSS/GPS bringup via `gpsd_client` (reads `gpsd`, not the serial port directly), targeting a SimpleRTK3B receiver. Publishes `/fix` + `/gps_path` + `map -> base_link` TF. See [gps_bringup/README.md](gps_bringup/README.md). |
| [`um982_driver`](um982_driver) | `ament_python` | GNSS velocity + dual-antenna heading from a Unicore UM982 (`/gnss/velocity`, `/gnss/heading`), via the `um982-driver` PyPI package. Does **not** publish `/fix` (gpsd_client owns it). See [um982_driver/README.md](um982_driver/README.md). |
| [`eod_av_launch`](eod_av_launch) | `ament_python` | Top-level orchestration: one launch per sensor plus a master `all_sensors.launch.py` that runs them all without duplicating any launch. See [Launching](#launching). |

## Requirements
- ROS 2 sourced (tested against Jazzy)
- `colcon`, `rosdep`
- Vendor SDKs / deps installed outside of rosdep:
  - **LUCID Arena SDK** for `arena_camera_node` — install via LUCID's installer first; the build looks for `/etc/ld.so.conf.d/Arena_SDK.conf` and fails fast if it's missing.
  - **`libpcap-dev`** for `ars430_ros_publisher` (`sudo apt install libpcap-dev`).
  - **`um982-driver`** (PyPI, GPL-3.0) for `um982_driver`: `pip install um982-driver` (no rosdep key, not installed by `colcon build`).
- `gpsd_client` for `gps_bringup` is a normal rosdep/apt dependency (`ros-<distro>-gpsd-client`).

## Build

These packages live at the repo root, so build the repo **as the `src/` of a
colcon workspace** (colcon expects packages under `src/`):

```bash
mkdir -p ~/eodav_ws/src
git clone <this-repo> ~/eodav_ws/src/EOD-AV     # or: cp -r EOD-AV ~/eodav_ws/src/
cd ~/eodav_ws

rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Launching

One launch per sensor (the 3 cameras count as one), plus a master and the
processing pipeline. **Every one takes `mode:=live|record` and
`rviz:=true|false`**, each with its own config in `eod_av_launch/rviz/`.

```bash
ros2 launch eod_av_launch cameras.launch.py      # 3x Triton  -> 3 Image displays
ros2 launch eod_av_launch lidar.launch.py        # Hesai      -> PointCloud2
ros2 launch eod_av_launch radar.launch.py        # ARS430     -> PointCloud2 + clusters
ros2 launch eod_av_launch gnss.launch.py         # RTK3B      -> /gps_path + TF
ros2 launch eod_av_launch all_sensors.launch.py  # everything -> ONE combined RViz

# the same four, as pure sources (this is what runs while recording)
ros2 launch eod_av_launch <any of the above> mode:=record
```

`mode` decides whether the derived nodes run; `rviz` only decides whether a
window opens. They are independent: `mode:=live rviz:=false` keeps the debayer
and the radar filters running headless, and `mode:=record` runs no derived node
at all regardless of `rviz`.

### Architecture: two modes

The package splits into **raw sources** and a **processing pipeline that doesn't
care where the data came from**:

```
SOURCES (what gets recorded)          PROCESSING (derived + view)
----------------------------------    ------------------------------
cameras -> .../image (bayer_rggb8)    optional debayer -> .../image_color
lidar   -> /lidar_points              (nothing: already usable)
radar   -> /unfiltered_radar_...      filters/DBSCAN -> clouds + clusters
gnss    -> /fix, /extended_fix        fix_to_path -> map->base_link TF
                                                     + /gps_path
                                      RViz

1. RECORD           = SOURCES                     -> rosbag
2. LIVE / PLAYBACK  = SOURCES or BAG + PROCESSING
```

**Live and playback are the same mode** — one pipeline, two possible sources —
so a replayed bag looks exactly like the live sensors.

```bash
ros2 launch eod_av_launch all_sensors.launch.py                    # live
ros2 launch eod_av_launch all_sensors.launch.py mode:=record       # recording
ros2 launch eod_av_launch processing.launch.py bag:=/path/to/bag   # playback
```

Each per-sensor launch is a raw source; with `mode:=live` it pulls in
`processing.launch.py` restricted to that sensor, so there is no duplicated
processing logic anywhere. With `mode:=record` it publishes only its raw
topics.

> **Images render in black and white, and that is expected.** RViz does not
> demosaic Bayer. For *verifying* the camera (framing, focus, exposure) that is
> enough. For colour use `rqt_image_view <topic>` (cv_bridge demosaics, no extra
> nodes) or `debayer:=true`.

> **The bag is self-contained.** The LiDAR cloud is recorded rather than the raw
> packets: packets are 4x smaller but need the sensor's angular correction file
> to be reconstructed, and a dataset that cannot be read without an external
> auxiliary file is fragile.
>
> What goes into the bag from TF is `/tf_static` — the extrinsics
> `base_link -> <each sensor>`, without which the bag does not fuse. The dynamic
> `map -> base_link` pose does **not**: `fix_to_path` is a derived node and does
> not run in record mode, so the pose (and `/gps_path`) are regenerated from the
> recorded `/fix` at playback, with the exact same node.

Master args: `mode` (`live`), `rviz` (`true`), `debayer` (`false`),
`enable_cameras`/`enable_lidar`/`enable_radar`/`enable_gnss` (all `true`),
`enable_um982_heading` (`false`), `pixelformat` (`bayer_rggb8`), `frame_rate`
(`12.0`), `radar_yaw_deg` (`0`).

> **Cameras stream Bayer, not RGB.** Measured on the bench: `rgb8` made the
> *firmware* interpolate and send 3 B/px (9.44 MB/frame), which **saturated the
> GigE link** (922 Mbps of ~950 usable) and capped the rate at ~10 fps with
> ±33 ms jitter. With `bayer_rggb8` (3.15 MB/frame) the link drops to ~30 %,
> the rate is a solid **12.009 Hz** and jitter falls to **0.64 ms**.

### Cameras — runtime parameters (auto/fixed gain & exposure, live resolution)

Each camera's gain and exposure can be **auto or fixed and changed while
streaming** (no restart), and resolution can be changed live:

```bash
ros2 param set /arena_cam_enp6s0 gain 13.0             # fixed gain (switches to manual)
ros2 param set /arena_cam_enp6s0 gain_auto Continuous  # back to auto-gain
ros2 param set /arena_cam_enp6s0 exposure_time 20000.0 # fixed exposure (us)
ros2 param set /arena_cam_enp6s0 exposure_auto Continuous
ros2 param set /arena_cam_enp6s0 width 1024            # live resolution (restarts the stream)
```

`gain_auto`/`exposure_auto` take `Off` | `Continuous` | `Once`. The launch
defaults to **auto** (`Continuous`) on all 3 cameras; the `gain`/`exposure_time`
values are only used once you switch a camera to `Off` (fixed).

The individual driver launches still work directly too
(`ros2 launch arena_camera_node three_cameras.launch.py`, `hesai_ros_driver
start.py`, `ars430_ros_publisher radar_live.launch.py`, `gps_bringup
gps.launch.py`).

### TF tree

Each sensor launch publishes its **own** static transforms (from the shared
helper `eod_av_launch/bringup.py`), so composing them in `all_sensors.launch.py`
yields one connected tree with no duplicated transforms and no separate
"static transforms" launch:

Each sensor launch publishes its **own** transform, and all of them hang off
**one single common reference, `base_link`** — so composing them in
`all_sensors.launch.py` gives one connected tree (verified for every
`enable_*` combination):

```
map ──(GNSS, dynamic)──> base_link ──┬──> hesai_lidar
                                     ├──> radar_fixed          (yaw 0 by default)
                                     └──> camera_enp{6,7,8}s0
```

`gnss.launch.py` sets `path_child_frame:=base_link`, so `map -> base_link` moves
the **whole rig as one rigid body** while `/gps_path` traces the trajectory
behind it. Without GNSS, `base_link` is simply the root and everything still
resolves.

> `all_sensors.rviz` therefore uses **Fixed Frame `map`** (the world) with the
> camera targeting `base_link`: the rig drives through the scene and the path
> stays drawn behind it. With Fixed Frame `base_link` the vehicle would sit
> pinned at the centre and the world would slide around it instead. If GNSS is
> off (or before the first fix) `map` doesn't exist — switch Fixed Frame to
> `base_link` in RViz's dropdown.

**Every transform is identity by default** — no sensor carries a rotation, so
each frame keeps RViz's default orientation (+X forward, +Y left, +Z up, per
REP-103) and no sensor is special-cased. Real mounting offsets/rotations go in
once measured.

**Calibrating live (opt-in).** `radar_tf_live` defaults to `false` — a plain
static identity transform, same as every other sensor. Set it to `true` and the
radar's transform comes from the `tf_tuner` node instead, whose parameters are
re-read every tick, so you can rotate/translate it **while it runs** and watch
RViz update:

```bash
ros2 launch eod_av_launch all_sensors.launch.py radar_tf_live:=true
ros2 param set /radar_tf yaw_deg -45.0   # rotate the radar's forward, live
ros2 param set /radar_tf x 0.35          # translate it too
```

Each change is logged, so once it lines up you copy the numbers into
`RADAR_YAW_DEG` / the extrinsics in `bringup.py` and go back to the static
default for recording (no interpolation/extrapolation issues on bag playback).

> **Translations are still unmeasured** — all x/y/z are `0` on purpose (frames
> overlap at the origin, which fails loudly instead of silently corrupting
> fusion). Fill in the real offsets in `eod_av_launch/bringup.py` before treating
> a recording as calibrated.

## Recording rosbags

This stack is high-bandwidth (3× 2048×1536 `rgb8` ≈ 9.4 MB/frame each, plus
LiDAR + radar). Run everything **headless** (`all_sensors.launch.py`, RViz off
by default) and see **[GRABAR_ROSBAGS.md](GRABAR_ROSBAGS.md)** for the full
anti-lag checklist (NVMe, MCAP storage, recording raw LiDAR/radar packets,
image compression, DDS buffers, PTP, drop monitoring) and a reference command.

## Repository layout

```
EOD-AV/
├── arena_camera_node/     # LUCID Arena camera driver (C++)
├── HesaiLidar_ROS_2.0/    # Hesai LiDAR driver, vendored from upstream
├── ars430_ros_publisher/  # Continental ARS430 radar driver (C++)
├── gps_bringup/           # GNSS/GPS bringup via gpsd_client
├── um982_driver/          # UM982 velocity + dual-antenna heading (Python)
├── eod_av_launch/         # per-sensor + all-sensors launch orchestration
└── setup/                 # host-level provisioning for PTP time sync — see setup/README.md
```

## Time synchronization (PTP)

This is a dataset-creation project: recordings from the 3 cameras, the LiDAR,
the radar, and the GNSS receiver only fuse correctly if every sensor timestamps
its data off the **same clock**. Each device free-runs on its own clock
otherwise, and their drift compounds over a recording session, corrupting
alignment. IEEE 1588 PTP ties them all to one shared, disciplined clock — that's
what `setup/` provisions.

[`setup/`](setup/) holds the host-side PTP provisioning script
(`setup_ptp_sync.sh`) — see [setup/README.md](setup/README.md) for what the
script does and why each part is necessary.

## License

Per-package: `gps_bringup`, `arena_camera_node`, `ars430_ros_publisher`, and
`eod_av_launch` are MIT; `um982_driver`'s own code is MIT while its `um982-driver`
runtime dependency is GPL-3.0 (kept out-of-tree, not vendored); `hesai_ros_driver`
(`HesaiLidar_ROS_2.0`) is vendored upstream under Hesai's BSD license — see each
package's `LICENSE`/`package.xml` for specifics.
