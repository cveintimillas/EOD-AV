from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

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
    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gps_bringup'),
                'launch',
                'gps.launch.py'   # ⚠️ si tu GPS usa otro nombre, lo ajustamos
            )
        )
    )

    return LaunchDescription([
        camera_launch,
        lidar_launch,
        gps_launch
    ])
