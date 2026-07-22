"""Live (or pcap-replay) radar pipeline for ROS2 Jazzy.

    ros2 launch ars430_ros_publisher radar_live.launch.py
    ros2 launch ars430_ros_publisher radar_live.launch.py raw:=true
    ros2 launch ars430_ros_publisher radar_live.launch.py pcap_file:=/path/radar_raw.pcap
    ros2 launch ars430_ros_publisher radar_live.launch.py iface:=enp3s0 port:=31122

Publishes:
    /unfiltered_radar_packet_<id>  (all decoded detections)
    /filtered_radar_packet_<id>    (after radar_processor filters)
    /radar_pointcloud_<id>         (filtered cloud)
    /radar_pointcloud_99           (raw cloud)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _nodes(context):
    radar_id = LaunchConfiguration('id').perform(context)
    return [
        Node(
            package='ars430_ros_publisher', executable='radar_publisher',
            name='radar_publisher', output='screen',
            parameters=[{
                'id': int(radar_id),
                'iface': LaunchConfiguration('iface'),
                'port': LaunchConfiguration('port'),
                'pcap_file': LaunchConfiguration('pcap_file'),
            }]),
        Node(
            package='ars430_ros_publisher', executable='radar_processor',
            name='radar_processor', output='screen',
            parameters=[{
                'id': int(radar_id),
                'raw': LaunchConfiguration('raw'),
                'snr_min_near': LaunchConfiguration('snr_min_near'),
                'snr_min_far': LaunchConfiguration('snr_min_far'),
                'velocity_min': LaunchConfiguration('velocity_min'),
                'range_min': LaunchConfiguration('range_min'),
                'range_max': LaunchConfiguration('range_max'),
            }]),
        Node(
            package='ars430_ros_publisher', executable='radar_visualizer',
            name='radar_visualizer_filtered',
            parameters=[{
                'input_topic': '/filtered_radar_packet_%s' % radar_id,
                'output_topic': '/radar_pointcloud_%s' % radar_id,
            }]),
        Node(
            package='ars430_ros_publisher', executable='radar_visualizer',
            name='radar_visualizer_raw',
            parameters=[{
                'input_topic': '/unfiltered_radar_packet_%s' % radar_id,
                'output_topic': '/radar_pointcloud_99',
            }]),
        # Tracker interno del radar (service 230): experimental, desactivado por
        # defecto tras el experimento de calibracion (ver README) — activar con
        # enable_tracker:=true
        *([Node(
            package='ars430_ros_publisher', executable='radar_objects',
            name='radar_objects', output='screen',
            parameters=[{
                'id': int(radar_id),
                'prob_min': LaunchConfiguration('prob_min'),
                'min_seen_scans': LaunchConfiguration('min_seen_scans'),
            }])] if LaunchConfiguration('enable_tracker').perform(context).lower() in ('true', '1') else []),
        Node(
            package='ars430_ros_publisher', executable='radar_clusters',
            name='radar_clusters', output='screen',
            parameters=[{
                'id': int(radar_id),
                'velocity_min': LaunchConfiguration('cluster_velocity_min'),
            }]),
        Node(
            package='ars430_ros_publisher', executable='radar_stats.py',
            name='radar_stats', output='screen',
            arguments=['-t', '/unfiltered_radar_packet_%s' % radar_id]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('id', default_value='1'),
        DeclareLaunchArgument('iface', default_value='enp9s0'),
        DeclareLaunchArgument('port', default_value='40000'),
        DeclareLaunchArgument('pcap_file', default_value="''",
                              description='replay a tcpdump capture instead of live sniffing'),
        DeclareLaunchArgument('raw', default_value='false',
                              description='true = processor forwards everything unfiltered'),
        DeclareLaunchArgument('snr_min_near', default_value='0.0'),
        DeclareLaunchArgument('snr_min_far', default_value='0.0'),
        DeclareLaunchArgument('velocity_min', default_value='0.0'),
        DeclareLaunchArgument('range_min', default_value='0.25'),
        DeclareLaunchArgument('range_max', default_value='100.0'),
        DeclareLaunchArgument('prob_min', default_value='50',
                              description='existence probability threshold for tracked objects'),
        DeclareLaunchArgument('min_seen_scans', default_value='3',
                              description='persistence: show a track after N consecutive scans'),
        DeclareLaunchArgument('cluster_velocity_min', default_value='0.3',
                              description='velocity gate for the DBSCAN cluster markers'),
        DeclareLaunchArgument('enable_tracker', default_value='false',
                              description='enable the experimental service-230 tracker markers'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('ars430_ros_publisher'), 'rviz', 'radar_diagnostic.rviz'])]),
        OpaqueFunction(function=_nodes),
    ])
