"""
Launch the UM982 raw PPK capture node standalone, gated off via IfCondition.

Shares /dev/um982_heading with um982_heading_node -- mutually exclusive on
that port, same rule as um982_heading_node vs gnss_heading_gpsd_node on
/gnss/heading. Do not run both against the same port at once.

Requires `configure_um982 --enable-ppk-raw` to have been run beforehand so
the receiver is actually emitting OBSVMB + ephemeris on this port -- this
launch file only starts the ROS-side capture, it never touches receiver
config.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Build the LaunchDescription for the um982_raw_log_node."""
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/um982_heading',
        description=(
            'Same dedicated serial link um982_heading_node uses -- mutually exclusive '
            'with it (see README). Requires configure_um982 --enable-ppk-raw to have '
            'been run beforehand.'
        ),
    )
    baud_arg = DeclareLaunchArgument('baud', default_value='115200')
    raw_log_enabled_arg = DeclareLaunchArgument(
        'raw_log_enabled', default_value='true',
        description=(
            'Gate for the whole node. Defaults true here because this launch file '
            'exists specifically to run raw capture -- the node parameter itself '
            'still defaults false as a fallback for bare `ros2 run`.'
        ),
    )
    raw_log_dir_arg = DeclareLaunchArgument(
        'raw_log_dir', default_value='/data/eod_av/gnss_raw',
        description=(
            'Placeholder -- point this at the actual SSD dataset directory for the '
            "session, e.g. alongside record_dataset.sh's -o output folder."
        ),
    )
    session_name_arg = DeclareLaunchArgument(
        'session_name', default_value='',
        description='Empty = autogenerate eodav_%Y%m%d_%H%M%S. Pass one explicitly to '
                    "match a specific bag session's name.",
    )

    raw_log_node = Node(
        package='um982_driver',
        executable='um982_raw_log_node',
        name='um982_raw_log_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('raw_log_enabled')),
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': ParameterValue(LaunchConfiguration('baud'), value_type=int),
            'raw_log_enabled': ParameterValue(
                LaunchConfiguration('raw_log_enabled'), value_type=bool),
            'raw_log_dir': LaunchConfiguration('raw_log_dir'),
            'session_name': LaunchConfiguration('session_name'),
        }],
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        raw_log_enabled_arg,
        raw_log_dir_arg,
        session_name_arg,
        raw_log_node,
    ])
