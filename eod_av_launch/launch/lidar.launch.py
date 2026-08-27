"""LiDAR Hesai -- FUENTE.

Publica `/lidar_points` (nube) directamente desde el driver, y eso es lo que se
graba. Decision deliberada: los paquetes UDP crudos pesan 4x menos, pero para
volver a convertirlos en nube hace falta el ARCHIVO DE CORRECCION angular del
sensor. Un dataset que no se puede leer sin un archivo auxiliar guardado aparte
es fragil a anios vista, y los 41 MB/s de diferencia ya no aprietan despues del
ahorro de las camaras (345 -> 113 MB/s).

Asi el bag es AUTOCONTENIDO: la nube se lee sin dependencias externas y el modo
live no necesita reexpandir nada.

    ros2 launch eod_av_launch lidar.launch.py                 # nube + RViz
    ros2 launch eod_av_launch lidar.launch.py mode:=record    # solo publicar
"""
import os

from ament_index_python.packages import get_package_share_directory
from eod_av_launch.bringup import (MODE_LIVE, if_live, if_record, lidar_tf,
                                   scoped, validate_mode)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'lidar_config', default_value='config.yaml',
            description="YAML del driver Hesai. "
                        "'config_raw_packets.yaml' graba los paquetes UDP "
                        "crudos sin parsear la nube (mucho menos CPU, y es "
                        "el modo de maxima fidelidad para dataset). "
                        "'config_recv_timestamp.yaml' es config.yaml con "
                        "use_timestamp_type:=1: sella con el reloj del host "
                        "(disciplinado por PPS) en vez del reloj del LiDAR, "
                        "que sigue en PTP Free Run."),
        DeclareLaunchArgument(
            'mode', default_value=MODE_LIVE,
            description='live (fuente + RViz) | record (solo /lidar_points, '
                        '/lidar_imu y /lidar_ptp, sin ventana).'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Abrir RViz con la nube. Solo aplica en live.'),
    ]

    # Cual YAML del driver usar. 'config.yaml' arma la nube en tiempo real;
    # 'config_raw_packets.yaml' publica los paquetes UDP crudos y NO parsea
    # (ver la cabecera de ese archivo: el parseo en vivo no da abasto y
    # /lidar_points sale con segundos de atraso).
    config = PathJoinSubstitution([
        get_package_share_directory('hesai_ros_driver'), 'config',
        LaunchConfiguration('lidar_config')])

    driver = Node(
        namespace='hesai_ros_driver',
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        output='screen',
        parameters=[{'config_path': config}])

    # La nube ya viene lista del driver, asi que aca el pipeline solo aporta
    # RViz. Se incluye igual (y no un rviz2 suelto) para que la vista del LiDAR
    # sea EXACTAMENTE la misma pieza en vivo y reproduciendo un bag.
    view = scoped(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('eod_av_launch'),
                         'launch', 'processing.launch.py')),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': 'lidar.rviz',
            'enable_cameras': 'false',
            'enable_radar': 'false',
            'enable_gnss': 'false',
        }.items()),
        condition=if_live())

    recording_note = LogInfo(
        condition=if_record(),
        msg='[lidar] mode:=record -- solo /lidar_points, /lidar_imu y '
            '/lidar_ptp. Sin RViz.')

    return LaunchDescription(
        args + [validate_mode(), driver, lidar_tf(), view, recording_note])
