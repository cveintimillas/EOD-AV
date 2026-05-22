from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyUSB0', description='Serial port for GPS')
    baud_arg = DeclareLaunchArgument('baud', default_value='115200', description='Baud rate')
    frame_arg = DeclareLaunchArgument('frame_id', default_value='gps_link', description='TF frame id for GPS')

    nmea_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='nmea_serial_driver',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'frame_id': LaunchConfiguration('frame_id')
        }]
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        frame_arg,
        nmea_node
    ])
