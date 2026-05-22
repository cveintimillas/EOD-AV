from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyUSB0', description='Serial port for GPS')
    baud_arg = DeclareLaunchArgument('baud', default_value='115200', description='Baud rate')
    frame_arg = DeclareLaunchArgument('frame_id', default_value='gps_link', description='TF frame id for GPS')

    serial_setup = ExecuteProcess(
        cmd=[
            'stty',
            '-F',
            LaunchConfiguration('port'),
            LaunchConfiguration('baud'),
            'raw',
            '-echo',
            '-echoe',
            '-echok',
            '-crtscts',
        ],
        output='screen'
    )

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

    start_driver_after_serial_setup = RegisterEventHandler(
        OnProcessExit(
            target_action=serial_setup,
            on_exit=[nmea_node]
        )
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        frame_arg,
        serial_setup,
        start_driver_after_serial_setup
    ])
