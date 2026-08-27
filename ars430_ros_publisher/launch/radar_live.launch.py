"""Live (or pcap-replay) radar pipeline for ROS2 Jazzy.

    ros2 launch ars430_ros_publisher radar_live.launch.py
    ros2 launch ars430_ros_publisher radar_live.launch.py raw:=true
    ros2 launch ars430_ros_publisher radar_live.launch.py pcap_file:=/path/radar_raw.pcap
    ros2 launch ars430_ros_publisher radar_live.launch.py iface:=enp3s0 port:=31122

    # recording: only the sniffer/decoder, no derived topics
    ros2 launch ars430_ros_publisher radar_live.launch.py processing:=false

Publishes:
    /unfiltered_radar_packet_<id>  (all decoded detections)   <-- the raw one
    /radar_objects_raw_<id>        (tracker object list)
  and, when processing:=true (default):
    /filtered_radar_packet_<id>    (after radar_processor filters)
    /radar_pointcloud_<id>         (filtered cloud)
    /radar_pointcloud_99           (raw cloud)
    /radar/cluster_markers         (DBSCAN person-sized cubes)

`processing:=false` keeps only radar_publisher. Everything the other nodes
produce is derived from /unfiltered_radar_packet_<id>, so it can be
regenerated at playback time -- during acquisition it is pure overhead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1')


def _nodes(context):
    radar_id = LaunchConfiguration('id').perform(context)

    # The sniffer/decoder: the only node that produces raw data.
    # capture:=false skips it, so the processing chain can be fed from a rosbag
    # instead of from the wire (playback).
    nodes = []
    if _flag(context, 'capture'):
        nodes.append(Node(
            package='ars430_ros_publisher', executable='radar_publisher',
            name='radar_publisher', output='screen',
            parameters=[{
                'id': int(radar_id),
                'iface': LaunchConfiguration('iface'),
                'port': LaunchConfiguration('port'),
                'pcap_file': LaunchConfiguration('pcap_file'),
            }]))

    # raw_cloud_only: la nube SIN FILTRAR y nada mas.
    #
    # Existe para el modo record del dataset. Ahi se quiere el crudo tal como
    # sale del sensor, pero /unfiltered_radar_packet_N es un tipo de mensaje
    # propio que un visor externo (Lichtblick, Foxglove) no sabe dibujar,
    # mientras que un PointCloud2 si. Esta bandera agrega EXACTAMENTE un nodo
    # -- el visualizador de la nube cruda -- sin el filtrado, sin DBSCAN y sin
    # estadisticas, que son derivados reconstruibles y cuestan CPU durante la
    # grabacion.
    #
    # No hay perdida de informacion: /radar_pointcloud_99 se arma del mismo
    # /unfiltered_radar_packet_N que se graba al lado.
    solo_nube_cruda = _flag(context, 'raw_cloud_only')

    if not _flag(context, 'processing') and not solo_nube_cruda:
        return nodes  # recording: raw only

    if solo_nube_cruda:
        nodes.append(Node(
            package='ars430_ros_publisher', executable='radar_visualizer',
            name='radar_visualizer_raw',
            parameters=[{
                'input_topic': '/unfiltered_radar_packet_%s' % radar_id,
                'output_topic': '/radar_pointcloud_99',
                'frame_id': LaunchConfiguration('frame_id'),
                # La TF base_link -> radar_fixed la publica eod_av_launch.
                'publish_tf': False,
            }]))
        return nodes

    nodes += [
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
                'frame_id': LaunchConfiguration('frame_id'),
                # This one owns base_link -> frame_id when publish_tf is true.
                'publish_tf': LaunchConfiguration('publish_tf'),
            }]),
        Node(
            package='ars430_ros_publisher', executable='radar_clusters',
            name='radar_clusters', output='screen',
            parameters=[{
                'id': int(radar_id),
                'velocity_min': LaunchConfiguration('cluster_velocity_min'),
            }]),
    ]

    # Second cloud with the UNFILTERED detections: useful to compare against
    # the filtered one, but it doubles the visualizer load -- opt-in.
    if _flag(context, 'raw_cloud'):
        nodes.append(Node(
            package='ars430_ros_publisher', executable='radar_visualizer',
            name='radar_visualizer_raw',
            parameters=[{
                'input_topic': '/unfiltered_radar_packet_%s' % radar_id,
                'output_topic': '/radar_pointcloud_99',
                'frame_id': LaunchConfiguration('frame_id'),
                # Never publish the TF from the raw visualizer -- the filtered
                # one (or eod_av_launch) already does, avoid duplicates.
                'publish_tf': False,
            }]))

    # Tracker interno del radar (service 230): experimental, desactivado por
    # defecto tras el experimento de calibracion (ver README).
    if _flag(context, 'enable_tracker'):
        nodes.append(Node(
            package='ars430_ros_publisher', executable='radar_objects',
            name='radar_objects', output='screen',
            parameters=[{
                'id': int(radar_id),
                'prob_min': LaunchConfiguration('prob_min'),
                'min_seen_scans': LaunchConfiguration('min_seen_scans'),
            }]))

    if _flag(context, 'stats'):
        nodes.append(Node(
            package='ars430_ros_publisher', executable='radar_stats.py',
            name='radar_stats', output='screen',
            arguments=['-t', '/unfiltered_radar_packet_%s' % radar_id]))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('id', default_value='1'),
        DeclareLaunchArgument('iface', default_value='enp9s0'),
        DeclareLaunchArgument('port', default_value='40000'),
        DeclareLaunchArgument('pcap_file', default_value="''",
                              description='replay a tcpdump capture instead of live sniffing'),
        DeclareLaunchArgument('processing', default_value='true',
                              description='false = only radar_publisher (recording); '
                                          'derived topics are regenerated at playback'),
        DeclareLaunchArgument('capture', default_value='true',
                              description='false = do not sniff the wire; feed the '
                                          'processing chain from a rosbag (playback)'),
        DeclareLaunchArgument('raw_cloud', default_value='true',
                              description='also publish the unfiltered cloud '
                                          '/radar_pointcloud_99'),
        DeclareLaunchArgument('raw_cloud_only', default_value='false',
                              description='publish ONLY /radar_pointcloud_99 '
                                          '(one node), skipping the filtering, '
                                          'DBSCAN and stats chain. For dataset '
                                          'recording: a PointCloud2 that any '
                                          'external viewer can draw, without '
                                          'spending CPU on derived topics.'),
        DeclareLaunchArgument('stats', default_value='true',
                              description='print detection statistics every 5 s'),
        DeclareLaunchArgument('raw', default_value='false',
                              description='true = processor forwards everything unfiltered'),
        DeclareLaunchArgument('snr_min_near', default_value='0.0'),
        DeclareLaunchArgument('snr_min_far', default_value='0.0'),
        DeclareLaunchArgument('velocity_min', default_value='0.0'),
        DeclareLaunchArgument('range_min', default_value='0.25'),
        DeclareLaunchArgument('range_max', default_value='100.0'),
        DeclareLaunchArgument('frame_id', default_value='radar_fixed',
                              description='TF frame for the radar cloud/markers.'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='Let the visualizer publish base_link -> frame_id. '
                                          'Set false when eod_av_launch owns the TF tree.'),
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
