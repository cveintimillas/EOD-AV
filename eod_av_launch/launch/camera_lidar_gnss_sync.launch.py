from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # um982_driver adds /gnss/velocity + /gnss/heading (dual-antenna) from the
    # UM982's own dedicated serial link (/dev/um982_heading, confirmed on
    # bench: COM3 @ 115200 -- see um982_driver/README.md). It never touches
    # /dev/gps_pps, so it's independent of gpsd_client's /fix. Defaults to
    # false here to match gps_bringup/launch/gps.launch.py's own default,
    # pending T3 field verification (rotate the antenna baseline stationary
    # and confirm /gnss/heading tracks it) -- flip to true once confirmed.
    enable_um982_heading_arg = DeclareLaunchArgument(
        'enable_um982_heading', default_value='false',
        description='Also launch um982_driver for /gnss/velocity + /gnss/heading.'
    )

    # ---------------- CAMERA ----------------
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('arena_camera_node'),
                'launch',
                'three_cameras.launch.py'
            )
        )
    )

    # ---------------- LIDAR ----------------
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hesai_ros_driver'),
                'launch',
                'start.py'
            )
        )
    )

    # ---------------- GPS ----------------
    # path_child_frame:='hesai_lidar' hace que fix_to_path publique la TF
    # dinamica map -> hesai_lidar (en vez de map -> gps_link), moviendo el
    # frame del lidar a la ultima posicion del path en tiempo real.
    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gps_bringup'),
                'launch',
                'gps.launch.py'   # ⚠️ si tu GPS usa otro nombre, lo ajustamos
            )
        ),
        launch_arguments={
            'path_child_frame': 'hesai_lidar',
            'enable_um982_heading': LaunchConfiguration('enable_um982_heading'),
        }.items()
    )

    return LaunchDescription([
        enable_um982_heading_arg,
        camera_launch,
        lidar_launch,
        gps_launch
    ])
