"""Master: los 4 sensores. DOS modos.

    ros2 launch eod_av_launch all_sensors.launch.py                # live
    ros2 launch eod_av_launch all_sensors.launch.py mode:=record   # grabar

ARQUITECTURA -- dos capas y dos modos:

    FUENTES (lo unico que corre al grabar)  PROCESAMIENTO (derivado + vista)
    ------------------------------------    --------------------------------
    camaras -> .../image (bayer_rggb8)      debayer opcional -> .../image_color
    lidar   -> /lidar_points                (nada: ya viene lista)
    radar   -> /unfiltered_radar_packet_1   filtros/DBSCAN -> nubes + clusters
    gnss    -> /fix, /extended_fix          fix_to_path -> TF map->base_link
                                                        -> /gps_path
                                            RViz

  1. RECORD    = FUENTES                      -> el usuario graba con
                                                 `record_dataset.sh`
  2. LIVE      = FUENTES + PROCESAMIENTO      (fuente: los sensores)
     PLAYBACK  = rosbag  + PROCESAMIENTO      (fuente: un bag)

LIVE y PLAYBACK son el MISMO modo: identico pipeline, distinta fuente. Por eso
un bag reproducido se ve exactamente igual que los sensores en vivo:

    ros2 bag play <bag>                                    # terminal 1
    ros2 launch eod_av_launch processing.launch.py         # terminal 2

En RECORD no corre casi ningun nodo derivado, ni siquiera fix_to_path: lo que se
omite se reconstruye despues desde el crudo.
  - detecciones filtradas y clusters del radar  <- /unfiltered_radar_packet_1
  - TF map->base_link y /gps_path               <- /fix
  - .../image_color                             <- .../image (bayer)

LA UNICA EXCEPCION ES EL RADAR (radar_cloud:=true, por defecto). Los demas
sensores publican tipos ESTANDAR que cualquier visor externo dibuja solo desde
el bag: sensor_msgs/Image, PointCloud2, NavSatFix. El radar no --
/unfiltered_radar_packet_N es un tipo propio del paquete, y en Lichtblick queda
como datos opacos. Por eso en record se agrega UN nodo que convierte ese mismo
crudo en /radar_pointcloud_99 (nube SIN filtrar). El crudo se graba igual, asi
que no se pierde nada ni se decide nada de forma irreversible.

El sistema NO inicia la grabacion por su cuenta: eso lo hace el usuario.
"""
import os

from ament_index_python.packages import (PackageNotFoundError,
                                         get_package_share_directory)
from eod_av_launch.bringup import (MODE_LIVE, MODE_RECORD, RADAR_YAW_DEG,
                                   resolve_mode, scoped)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _bringup(context):
    this = get_package_share_directory('eod_av_launch')
    mode = resolve_mode(context)
    recording = mode == MODE_RECORD

    def include(name, enable_arg, extra=None):
        # Los bring-up entran SIEMPRE como fuentes puras (mode:='record'): el
        # procesamiento lo agrega este archivo una sola vez, mas abajo. Si les
        # pasara mode:='live', cada uno incluiria su propio
        # processing.launch.py y terminarian corriendo cuatro RViz y cuatro
        # copias de la cadena de radar. El mode que pide el usuario decide una
        # sola cosa aca: si ademas de las fuentes se incluye o no el pipeline.
        launch_arguments = {'mode': MODE_RECORD, 'rviz': 'false'}
        if extra:
            launch_arguments.update(extra)
        return scoped(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(this, 'launch', name)),
                launch_arguments=launch_arguments.items()),
            condition=IfCondition(LaunchConfiguration(enable_arg)))

    actions = [
        LogInfo(msg='[eod_av_launch] modo = ' + mode),
        # ---- FUENTES: identicas en los dos modos ------------------------
        include('cameras.launch.py', 'enable_cameras', {
            'pixelformat': LaunchConfiguration('pixelformat'),
            'frame_rate': LaunchConfiguration('frame_rate'),
            'gain_auto': LaunchConfiguration('gain_auto'),
            'exposure_auto': LaunchConfiguration('exposure_auto'),
        }),
        include('lidar.launch.py', 'enable_lidar', {
            'lidar_config': LaunchConfiguration('lidar_config'),
        }),
        include('radar.launch.py', 'enable_radar', {
            'radar_yaw_deg': LaunchConfiguration('radar_yaw_deg'),
            'radar_tf_live': LaunchConfiguration('radar_tf_live'),
            # Solo en record. En live la nube la publica processing.launch.py,
            # y pedirla dos veces deja dos nodos sobre /radar_pointcloud_99.
            'radar_cloud': (LaunchConfiguration('radar_cloud')
                            if recording else 'false'),
        }),
        include('gnss.launch.py', 'enable_gnss', {
            'enable_um982_heading': LaunchConfiguration('enable_um982_heading'),
            'enable_gnss_heading': LaunchConfiguration('enable_gnss_heading'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'use_gps_time': LaunchConfiguration('use_gps_time'),
        }),
    ]

    # sync_check corre en LOS DOS MODOS, igual que gnss_watchdog: no publica
    # ningun topico (solo loguea una tabla), asi que no ensucia el bag, y
    # durante una grabacion larga es justo cuando mas vale saber si algun reloj
    # se esta yendo. Apagado por defecto para no agregar ruido al log.
    #
    # La disponibilidad del paquete se resuelve ACA, no con un IfCondition, y el
    # motivo es concreto: si `eod_av_tools` no esta compilado, launch levanta
    # PackageNotFoundError al construir la descripcion y se cae TODO el bringup
    # -- las camaras, el LiDAR, el radar y el GNSS reciben SIGINT por culpa de
    # un nodo de diagnostico opcional. Paso en campo. Una herramienta de
    # verificacion no puede tumbar la adquisicion.
    if context.perform_substitution(
            LaunchConfiguration('sync_check')).lower() in ('true', '1'):
        try:
            get_package_share_directory('eod_av_tools')
            actions.append(Node(
                package='eod_av_tools', executable='sync_check',
                name='sync_check', output='screen',
                parameters=[{
                    'report_period': ParameterValue(
                        LaunchConfiguration('sync_check_period'),
                        value_type=float),
                }]))
        except PackageNotFoundError:
            actions.append(LogInfo(
                msg='[eod_av_launch] sync_check:=true pero el paquete '
                    'eod_av_tools no esta compilado. Sigo SIN el (los sensores '
                    'no dependen de el). Para tenerlo:  colcon build '
                    '--packages-select eod_av_tools && source install/setup.bash'))

    if recording:
        actions.append(LogInfo(
            msg='[eod_av_launch] record: datos CRUDOS. La unica excepcion es el '
                'radar, que ademas publica /radar_pointcloud_99 (nube sin '
                'filtrar) porque su mensaje crudo es de tipo propio y un visor '
                'externo no lo dibuja. Sin RViz, sin fix_to_path, sin filtros. '
                'Graba con `ros2 run eod_av_launch record_dataset.sh`. Para ver '
                'lo derivado despues: `processing.launch.py bag:=<bag>`.'))
        return actions

    # ---- PROCESAMIENTO: solo en live -----------------------------------
    actions.append(scoped(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(this, 'launch', 'processing.launch.py')),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': 'all_sensors.rviz',
            'debayer': LaunchConfiguration('debayer'),
            'enable_cameras': LaunchConfiguration('enable_cameras'),
            'enable_radar': LaunchConfiguration('enable_radar'),
            # El GNSS entra al pipeline igual que en playback: fix_to_path lo
            # lanza SIEMPRE processing, nunca el bring-up. Asi el grafo de
            # procesamiento es identico con sensores o con un bag.
            'enable_gnss': LaunchConfiguration('enable_gnss'),
            'fix_to_path': LaunchConfiguration('fix_to_path'),
            'publish_path': LaunchConfiguration('publish_path'),
        }.items())))
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'mode', default_value=MODE_LIVE,
            description='live (fuentes + procesamiento + RViz) | '
                        'record (solo fuentes, crudo, headless)'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Solo aplica en live.'),
        DeclareLaunchArgument(
            'debayer', default_value='false',
            description='true: .../image_color para ver color en RViz. Por '
                        'defecto B/N (alcanza para verificar); para color '
                        'tambien sirve rqt_image_view sin nodos extra.'),
        DeclareLaunchArgument('enable_cameras', default_value='true'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('enable_radar', default_value='true'),
        DeclareLaunchArgument('enable_gnss', default_value='true'),
        DeclareLaunchArgument('enable_um982_heading', default_value='false'),
        DeclareLaunchArgument(
            'enable_gnss_heading', default_value='true',
            description='/gnss/heading y /gnss/velocity leyendo las $GNHPR '
                        'que ya pasan por gpsd. Sin hardware extra. '
                        'Excluyente con enable_um982_heading.'),
        DeclareLaunchArgument(
            'fix_to_path', default_value='true',
            description='Solo en live: TF map->base_link desde /fix. '
                        'En record nunca corre (es dato derivado).'),
        DeclareLaunchArgument(
            'publish_path', default_value='true',
            description='Solo en live: ademas de la TF, publicar /gps_path.'),
        # camaras
        DeclareLaunchArgument('pixelformat', default_value='bayer_rggb8'),
        DeclareLaunchArgument('frame_rate', default_value='12.0'),
        DeclareLaunchArgument('gain_auto', default_value='Continuous'),
        DeclareLaunchArgument('exposure_auto', default_value='Continuous'),
        # radar
        DeclareLaunchArgument('radar_yaw_deg', default_value=str(RADAR_YAW_DEG)),
        DeclareLaunchArgument('radar_tf_live', default_value='false'),
        DeclareLaunchArgument(
            'radar_cloud', default_value='true',
            description='Solo en record: publicar tambien /radar_pointcloud_99, '
                        'la nube del radar SIN filtrar. Es un PointCloud2 que '
                        'cualquier visor dibuja, a diferencia del mensaje crudo '
                        'del radar. Un solo nodo extra.'),
        DeclareLaunchArgument(
            'publish_rate', default_value='10',
            description='Hz a los que gpsd_client publica /fix. Este receptor va '
                        'a 10 Hz (verificado en el NMEA): por debajo se '
                        'descartan fixes reales.'),
        DeclareLaunchArgument(
            'use_gps_time', default_value='true',
            description='true (recomendado): header.stamp de /fix = epoca GNSS '
                        'real. false: = hora de llegada, que esconde la latencia '
                        'adentro del dato. Ver gps_bringup/launch/gps.launch.py.'),
        DeclareLaunchArgument(
            'lidar_config', default_value='config.yaml',
            description="YAML del driver Hesai. 'config_raw_packets.yaml' graba "
                        'los paquetes UDP crudos sin parsear la nube: mucho menos '
                        'CPU y es el modo de maxima fidelidad para dataset. '
                        "'config_recv_timestamp.yaml' es config.yaml con "
                        'use_timestamp_type:=1: sella con el reloj del host '
                        '(disciplinado por PPS) en vez del reloj del LiDAR, que '
                        'sigue en PTP Free Run.'),
        DeclareLaunchArgument(
            'sync_check', default_value='false',
            description='Nodo que mide el desfase real de header.stamp de cada '
                        'topico (eod_av_tools/sync_check). No publica nada; '
                        'corre en los dos modos. Ver eod_av_tools/README.md.'),
        DeclareLaunchArgument(
            'sync_check_period', default_value='15.0',
            description='Cada cuantos segundos sync_check imprime la tabla.'),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_bringup)])
