"""Static TF tree for EOD-AV: base_link -> every sensor frame.

Diagnosed via Lichtblick/rosbag2 on a recorded bag (rosbag2_2026_07_17-15_55_19):
/tf_static only ever had ONE transform (base_link -> radar_fixed, identity --
0 translation, 0 rotation), and hesai_lidar had no transform to base_link at
all ("Missing transform from frame hesai_lidar to frame base_link"). The
existing base_link -> radar_fixed transform was not found declared anywhere
in this repo, in the separate ars430_ros_publisher radar workspace
(~/radar_ws), or in any systemd unit -- it was almost certainly published by
a one-off manual `ros2 run tf2_ros static_transform_publisher` invocation
that was never committed anywhere. This file is the first committed home for
any of these transforms.

*** ALL VALUES BELOW ARE UNMEASURED PLACEHOLDERS (0 translation, 0 rotation) ***
Every entry needs real x/y/z (meters) and roll/pitch/yaw (radians) from
physical measurement (tape measure / CAD, relative to base_link's origin)
before the next recording is treated as calibrated. Do not fill these in
with "reasonable-looking" guessed numbers -- an uncalibrated identity
transform is an obvious placeholder that fails loudly (frames overlap at the
origin); a plausible-looking guessed number is not, and would corrupt
sensor fusion/calibration silently.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def _static_tf(name, child_frame_id):
    # TODO(measure): replace x/y/z/roll/pitch/yaw with the real measurement
    # for this sensor before the next recording. All zeros = unmeasured.
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', child_frame_id,
        ],
    )


def generate_launch_description():
    return LaunchDescription([
        # Pre-existing before this file: base_link -> radar_fixed. Confirmed
        # identity/unmeasured (see module docstring) -- not a real value from
        # any CAD/config file found in this repo or ~/radar_ws.
        _static_tf('static_tf_radar_fixed', 'radar_fixed'),

        # New: hesai_lidar had NO transform to base_link before this file
        # (confirmed absent from /tf_static and flagged by Lichtblick).
        _static_tf('static_tf_hesai_lidar', 'hesai_lidar'),

        # New: the 3 Tritons. Frame names match arena_camera_node's 'frame_id'
        # parameter as set in arena_camera_node/launch/three_cameras.launch.py
        # (camera_enp6s0/camera_enp7s0/camera_enp8s0 -- enp6s0/7s0/8s0 after
        # the PC swap, matches setup/setup_ptp_sync.sh's CAM1/2/3_IFACE) --
        # previously each camera's image messages carried Arena SDK's
        # per-frame counter (e.g. "247", "822"...) as frame_id instead of a
        # fixed frame name, which is fixed separately in ArenaCameraNode.cpp.
        _static_tf('static_tf_camera_enp6s0', 'camera_enp6s0'),
        _static_tf('static_tf_camera_enp7s0', 'camera_enp7s0'),
        _static_tf('static_tf_camera_enp8s0', 'camera_enp8s0'),
    ])
