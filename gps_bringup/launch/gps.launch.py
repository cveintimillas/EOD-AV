import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    gpsd_host_arg = DeclareLaunchArgument(
        'gpsd_host', default_value='localhost',
        description='Host running gpsd (gpsd already owns the serial port, see setup/setup_ptp_sync.sh)'
    )
    gpsd_port_arg = DeclareLaunchArgument(
        'gpsd_port', default_value='2947', description='gpsd TCP port'
    )
    frame_arg = DeclareLaunchArgument('frame_id', default_value='gps_link', description='TF frame id for GPS')
    path_child_frame_arg = DeclareLaunchArgument(
        'path_child_frame',
        default_value='gps_link',
        description="Frame that fix_to_path moves to the last fix (map -> this frame). "
                    "Set to the lidar/base frame to make it follow the GPS trail."
    )

    # um982_driver adds velocity + dual-antenna heading, which gpsd_client can't
    # parse (proprietary PVTSLNA/BESTNAVA/HPR). It does NOT replace gpsd_client:
    # /fix keeps coming from gpsd_client with PPS-disciplined timestamps for the
    # PTP pipeline (see setup/setup_ptp_sync.sh) -- um982_driver reads its own,
    # separate serial link (see its udev rule template) so it never contends
    # with gpsd for /dev/gps_pps. Default off until that second link and its
    # udev rule are confirmed on the bench; set enable_um982_heading:=true once
    # validated in the field.
    enable_um982_heading_arg = DeclareLaunchArgument(
        'enable_um982_heading', default_value='false',
        description='Launch um982_driver for /gnss/velocity + /gnss/heading (needs its own '
                     'dedicated serial link to the UM982, separate from gpsd -- see '
                     'um982_driver/udev/99-eodav-um982-heading.rules.template).'
    )
    um982_heading_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('um982_driver'), 'launch', 'um982_heading.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('enable_um982_heading')),
        launch_arguments={'frame_id': LaunchConfiguration('frame_id')}.items(),
    )

    # gpsd (levantado por setup/setup_ptp_sync.sh) ya tiene abierto el puerto
    # serie del simpleRTK3B para alimentar el refclock NMEA+PPS de chrony. Este
    # nodo se conecta a gpsd por TCP en vez de reabrir el ttyUSB directamente,
    # evitando que dos procesos lean el mismo puerto serie a la vez.
    gpsd_client_container = ComposableNodeContainer(
        name='gpsd_client_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gpsd_client',
                plugin='gpsd_client::GPSDClientComponent',
                name='gpsd_client',
                parameters=[{
                    'host': LaunchConfiguration('gpsd_host'),
                    'port': LaunchConfiguration('gpsd_port'),
                    'use_gps_time': True,
                    'check_fix_by_variance': False,
                    'frame_id': LaunchConfiguration('frame_id'),
                    'publish_rate': 10,
                }],
            ),
        ],
        output='screen',
    )

    fix_to_path_node = Node(
        package='gps_bringup',
        executable='fix_to_path.py',
        name='fix_to_path',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'fix_topic': '/fix',
            'child_frame': LaunchConfiguration('path_child_frame')
        }]
    )

    return LaunchDescription([
        gpsd_host_arg,
        gpsd_port_arg,
        frame_arg,
        path_child_frame_arg,
        enable_um982_heading_arg,
        gpsd_client_container,
        fix_to_path_node,
        um982_heading_launch,
    ])
