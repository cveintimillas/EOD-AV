"""GNSS/GPS (simpleRTK3B via gpsd_client) -- FUENTE.

Publica `/fix` y `/extended_fix` y, opcionalmente, `/gnss/velocity` +
`/gnss/heading` del UM982.

Este launch es FUENTE PURA: publica solo lo que sale del receptor. Ni la TF
dinamica `map -> base_link` ni `/gps_path` se generan aca -- ambas son
DERIVADAS de `/fix` y las produce fix_to_path, que vive en
processing.launch.py y corre igual en vivo que reproduciendo un bag.

Si que se publica la TF ESTATICA `base_link -> gps_link`: no es un dato
derivado sino una extrinseca del vehiculo (donde esta montada la antena), y
gpsd_client estampa `/fix` con ese frame_id. Sin ella ese frame no existe en el
arbol.

    ros2 launch eod_av_launch gnss.launch.py                 # + trayectoria + RViz
    ros2 launch eod_av_launch gnss.launch.py mode:=record    # solo /fix
"""
import os

from ament_index_python.packages import get_package_share_directory
from eod_av_launch.bringup import (BASE_FRAME, MODE_LIVE, gnss_tf, if_live,
                                   if_record, scoped, validate_mode)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'mode', default_value=MODE_LIVE,
            description='live (fuente + fix_to_path + RViz) | record (solo /fix '
                        'y /extended_fix; sin TF map->base_link, sin /gps_path).'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Abrir RViz con /gps_path + TF. Solo aplica en live.'),
        DeclareLaunchArgument(
            'enable_um982_heading', default_value='false',
            description='Lanzar um982_driver para /gnss/velocity + /gnss/heading.'),
        DeclareLaunchArgument(
            'enable_gnss_heading', default_value='true',
            description='/gnss/heading y /gnss/velocity leyendo las $GNHPR '
                        'que ya pasan por gpsd. Sin hardware extra. '
                        'Excluyente con enable_um982_heading.'),
        DeclareLaunchArgument(
            'path_child_frame', default_value=BASE_FRAME,
            description='Frame que fix_to_path mueve (map -> este frame).'),
        DeclareLaunchArgument(
            'fix_to_path', default_value='true',
            description='Solo en live: TF map->base_link desde /fix. Se pasa a '
                        'processing.launch.py.'),
        DeclareLaunchArgument(
            'publish_path', default_value='true',
            description='Solo en live: ademas de la TF, publicar /gps_path.'),
        DeclareLaunchArgument(
            'path_min_distance', default_value='0.2',
            description='Solo en live: umbral de movimiento del Path (m). Ver '
                        'processing.launch.py.'),
        DeclareLaunchArgument(
            'publish_rate', default_value='10',
            description='Hz a los que gpsd_client publica /fix. Este receptor va '
                        'a 10 Hz: por debajo se descartan fixes reales. Ver '
                        'gps_bringup/launch/gps.launch.py.'),
        DeclareLaunchArgument(
            'use_gps_time', default_value='true',
            description='true (recomendado): header.stamp de /fix = epoca GNSS '
                        'real. false: = hora de llegada, que esconde la latencia '
                        'adentro del dato. Ver gps_bringup/launch/gps.launch.py.'),
        DeclareLaunchArgument(
            'gnss_watchdog', default_value='true',
            description='Aviso por consola del estado del GNSS (sin /fix, /fix '
                        'sin lock, /fix con lock). No publica ningun topico, '
                        'corre en los dos modos.'),
    ]

    source = scoped(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gps_bringup'),
                'launch', 'gps.launch.py')),
        launch_arguments={
            # fuente pura: fix_to_path lo lanza processing, nunca este bring-up
            'enable_path': 'false',
            'enable_um982_heading': LaunchConfiguration('enable_um982_heading'),
            'enable_gnss_heading': LaunchConfiguration('enable_gnss_heading'),
            'path_child_frame': LaunchConfiguration('path_child_frame'),
            'gnss_watchdog': LaunchConfiguration('gnss_watchdog'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'use_gps_time': LaunchConfiguration('use_gps_time'),
        }.items()))

    # enable_gnss:='true' -- este include es el UNICO que puede levantar
    # fix_to_path en todo el flujo, porque el bring-up de arriba entra con
    # enable_path:='false'. Antes iba en 'false' y el resultado era que
    # `gnss.launch.py` en vivo abria gnss.rviz mostrando /gps_path y una TF
    # map->base_link que no publicaba nadie: la ventana quedaba vacia.
    processing = scoped(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('eod_av_launch'),
                         'launch', 'processing.launch.py')),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': 'gnss.rviz',
            'enable_cameras': 'false',
            'enable_radar': 'false',
            'enable_gnss': 'true',
            'fix_to_path': LaunchConfiguration('fix_to_path'),
            'publish_path': LaunchConfiguration('publish_path'),
            'path_min_distance': LaunchConfiguration('path_min_distance'),
        }.items()),
        condition=if_live())

    recording_note = LogInfo(
        condition=if_record(),
        msg='[gnss] mode:=record -- solo /fix y /extended_fix. Sin fix_to_path, '
            'sin /gps_path, sin RViz.')

    return LaunchDescription(
        args + [validate_mode(), source, gnss_tf(), processing, recording_note])
