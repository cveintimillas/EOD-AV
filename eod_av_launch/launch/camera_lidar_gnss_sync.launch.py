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
            'path_child_frame': 'hesai_lidar'
        }.items()
    )

    return LaunchDescription([
        camera_launch,
        lidar_launch,
        gps_launch
    ])
