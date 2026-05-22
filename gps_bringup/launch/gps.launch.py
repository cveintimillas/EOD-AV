from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Ejecuta el wrapper Python del GPS
        ExecuteProcess(
            cmd=['/usr/bin/python3', '/home/cveintimillas/ros2_ws/src/gps_bringup/nodes/gps_node.py'],
            output='screen'
        )
    ])
