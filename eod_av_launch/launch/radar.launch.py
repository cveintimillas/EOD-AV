"""Radar Continental ARS430 (RDI v2, enp9s0) -- FUENTE de datos crudos.

Corre SOLO `radar_publisher`: sniffer libpcap + decoder v2 ->
`/unfiltered_radar_packet_1`. El filtrado, las nubes PointCloud2 y los clusters
NO se generan aca: los produce processing.launch.py a partir de ese mismo
topico, venga de la red o de un rosbag.

FRAME: radar_fixed cuelga de base_link sin rotacion por defecto. El frente se
ajusta en caliente con radar_tf_live:=true y
`ros2 param set /radar_tf yaw_deg <grados>`.

    ros2 launch eod_av_launch radar.launch.py                 # crudo + pipeline + RViz
    ros2 launch eod_av_launch radar.launch.py mode:=record    # solo crudo (grabar)
    ros2 launch eod_av_launch radar.launch.py pcap_file:=/ruta/captura.pcap
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from eod_av_launch.bringup import (MODE_LIVE, MODE_RECORD, RADAR_PARENT_FRAME,
                                   RADAR_YAW_DEG, flag, if_live, if_record,
                                   radar_tf, radar_tf_tunable, resolve_mode,
                                   scoped, validate_mode)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _radar_tf(context):
    yaw_deg = float(LaunchConfiguration('radar_yaw_deg').perform(context))
    parent = LaunchConfiguration('radar_parent').perform(context)
    if flag(context, 'radar_tf_live'):
        return [radar_tf_tunable(yaw_deg, parent=parent)]
    return [radar_tf(math.radians(yaw_deg), parent=parent)]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'mode', default_value=MODE_LIVE,
            description='live (sniffer + filtros/nubes/clusters) | record '
                        '(SOLO /unfiltered_radar_packet_N, sin nada derivado).'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Abrir RViz. Solo aplica en live; el crudo por si solo '
                        'no se dibuja, asi que sin pipeline no hay nada que ver.'),
        DeclareLaunchArgument(
            'radar_yaw_deg', default_value=str(RADAR_YAW_DEG),
            description='Rotacion radar_parent -> radar_fixed en GRADOS.'),
        DeclareLaunchArgument(
            'radar_parent', default_value=RADAR_PARENT_FRAME,
            description='Frame padre del radar (base_link por defecto).'),
        DeclareLaunchArgument(
            'radar_tf_live', default_value='false',
            description='true: TF ajustable en caliente via '
                        '`ros2 param set /radar_tf yaw_deg <grados>`.'),
        DeclareLaunchArgument('iface', default_value='enp9s0'),
        DeclareLaunchArgument(
            'pcap_file', default_value="''",
            description='Reproducir un pcap en vez de capturar en vivo.'),
        DeclareLaunchArgument(
            'radar_cloud', default_value='true',
            description='Publicar /radar_pointcloud_99, la nube SIN filtrar, '
                        'ademas del crudo. Es un PointCloud2 estandar que un '
                        'visor externo dibuja sin conocer los mensajes propios '
                        'del radar. Un solo nodo; no arrastra filtros ni '
                        'clusters. En live lo publica igual el pipeline.'),
    ]

    # Fuente: el sniffer/decoder, y opcionalmente el visualizador de la nube
    # cruda.
    #
    # POR QUE EL RADAR ES LA EXCEPCION AL "SOLO CRUDO" DE record
    # Los demas sensores publican tipos estandar que cualquier visor externo
    # dibuja solo: sensor_msgs/Image, sensor_msgs/PointCloud2, NavSatFix. El
    # radar no: /unfiltered_radar_packet_N es un tipo propio
    # (ars430_ros_publisher/msg/RadarPacket) que Lichtblick no sabe interpretar,
    # asi que en el bag queda como datos opacos.
    #
    # radar_cloud:=true agrega UN nodo que convierte ese mismo crudo en un
    # PointCloud2 (/radar_pointcloud_99, sin filtrar). No reemplaza al crudo --
    # los dos se graban -- y no arrastra el filtrado, DBSCAN ni las
    # estadisticas, que si son derivados y cuestan CPU mientras se graba.
    # Se resuelve en un OpaqueFunction porque `raw_cloud_only` depende del MODO:
    # en live, processing.launch.py ya levanta radar_visualizer_raw, asi que
    # pedirlo tambien aca dejaria DOS nodos publicando /radar_pointcloud_99.
    def _source(context):
        recording = resolve_mode(context) == MODE_RECORD
        return [scoped(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ars430_ros_publisher'),
                    'launch', 'radar_live.launch.py')),
            launch_arguments={
                'rviz': 'false',
                'processing': 'false',  # el pipeline vive en processing.launch.py
                # Solo en record: en live lo aporta el pipeline.
                'raw_cloud_only': (LaunchConfiguration('radar_cloud').perform(context)
                                   if recording else 'false'),
                'iface': LaunchConfiguration('iface'),
                'pcap_file': LaunchConfiguration('pcap_file'),
            }.items()))]

    source = OpaqueFunction(function=_source)

    # El pipeline depende del MODO, no de rviz: `mode:=live rviz:=false` deja
    # los filtros y clusters corriendo sin ventana.
    processing = scoped(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('eod_av_launch'),
                         'launch', 'processing.launch.py')),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': 'radar.rviz',
            'enable_cameras': 'false',
            'enable_radar': 'true',
            'enable_gnss': 'false',
        }.items()),
        condition=if_live())

    recording_note = LogInfo(
        condition=if_record(),
        msg='[radar] mode:=record -- solo /unfiltered_radar_packet_N y '
            '/radar_objects_raw_N. Sin filtros, sin clusters, sin RViz.')

    return LaunchDescription(
        args + [validate_mode(), source, OpaqueFunction(function=_radar_tf),
                processing, recording_note])
