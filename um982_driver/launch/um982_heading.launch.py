"""Launch the UM982 velocity/heading node standalone, with its serial port as an argument."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the LaunchDescription for the um982_heading_node."""
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/um982_heading',
        description=(
            'Dedicated serial link for UM982 PVTSLNA/BESTNAVA/HPR logs -- separate from '
            "gpsd's /dev/gps_pps (PTP/PPS pipeline, do not share). This is a udev-symlink "
            'placeholder: add a rule like /etc/udev/rules.d/99-eodav-gps.rules for this '
            "device's USB serial number once confirmed on the bench (see README)."
        ),
    )
    baud_arg = DeclareLaunchArgument('baud', default_value='115200')
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='gps_link',
        description='Same TF frame gpsd_client/fix_to_path already use, to keep one GPS frame.'
    )
    publish_rate_arg = DeclareLaunchArgument('publish_rate_hz', default_value='10.0')

    heading_node = Node(
        package='um982_driver',
        executable='um982_heading_node',
        name='um982_heading_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
        }],
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        frame_id_arg,
        publish_rate_arg,
        heading_node,
    ])
