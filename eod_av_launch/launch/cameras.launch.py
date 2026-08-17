"""Las 3 camaras LUCID Triton -- FUENTE de datos crudos (cuentan como 1 sensor).

Publican `bayer_rggb8` (1 B/px, el dato del sensor sin interpolar). El paso a
color NO se hace aca: lo hace processing.launch.py, igual que si las imagenes
vinieran de un rosbag.

Por que Bayer: con rgb8 el firmware interpolaba y mandaba 3 B/px (9,44
MB/frame), saturando el enlace GigE (922 de ~950 Mbps) y limitando a ~10 fps con
jitter de +-33 ms. Con Bayer son 3,15 MB/frame: mismo contenido, mas crudo, y el
enlace baja al ~30%. El tope de `frame_rate` es lo que impide que la camara se
coma ese margen recien liberado subiendo sola hacia ~36 fps.

OJO: RViz NO demosaica Bayer -- muestra las imagenes en BLANCO Y NEGRO si se
suscribe al topico crudo. Es suficiente para verificar encuadre, foco y
exposicion. Para color: `rqt_image_view`, o debayer:=true.

    ros2 launch eod_av_launch cameras.launch.py                  # live + RViz
    ros2 launch eod_av_launch cameras.launch.py mode:=record     # solo crudo
    ros2 launch eod_av_launch cameras.launch.py mode:=live rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from eod_av_launch.bringup import (MODE_LIVE, camera_tfs, if_live, if_record,
                                   scoped, validate_mode)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'mode', default_value=MODE_LIVE,
            description='live (fuentes + procesamiento) | record (SOLO el crudo: '
                        'sin debayer, sin RViz). El sistema no inicia la grabacion: '
                        'eso lo hace `ros2 bag record -a` a mano.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Abrir RViz. Solo aplica en live; en record nunca se abre.'),
        DeclareLaunchArgument(
            'pixelformat', default_value='bayer_rggb8',
            description='bayer_rggb8 (crudo, 1 B/px) | rgb8 (3 B/px, satura GigE). '
                        'Si los colores salen cruzados probar bayer_bggr8 / '
                        'bayer_gbrg8 / bayer_grbg8.'),
        DeclareLaunchArgument(
            'frame_rate', default_value='12.0',
            description='Tope de fps. <=0 = sin tope (la camara se come el margen).'),
        DeclareLaunchArgument('gain_auto', default_value='Continuous'),
        DeclareLaunchArgument('exposure_auto', default_value='Continuous'),
        DeclareLaunchArgument(
            'debayer', default_value='false',
            description='Solo en live: publicar tambien .../image_color. Por '
                        'defecto no: RViz muestra el Bayer en B/N, que alcanza '
                        'para verificar. Para color: rqt_image_view <topico>.'),
    ]

    cameras = scoped(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('arena_camera_node'),
                'launch', 'three_cameras.launch.py')),
        launch_arguments={
            'pixelformat': LaunchConfiguration('pixelformat'),
            'frame_rate': LaunchConfiguration('frame_rate'),
            'gain_auto': LaunchConfiguration('gain_auto'),
            'exposure_auto': LaunchConfiguration('exposure_auto'),
        }.items()))

    # El procesamiento depende del MODO, no de rviz: asi `mode:=live
    # rviz:=false` deja el debayer corriendo sin ventana, y `mode:=record` no
    # levanta ningun nodo derivado aunque alguien pase rviz:=true.
    processing = scoped(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('eod_av_launch'),
                         'launch', 'processing.launch.py')),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': 'cameras.rviz',
            'debayer': LaunchConfiguration('debayer'),
            'enable_cameras': 'true',
            'enable_radar': 'false',
            'enable_gnss': 'false',
        }.items()),
        condition=if_live())

    recording_note = LogInfo(
        condition=if_record(),
        msg='[cameras] mode:=record -- solo las 3 fuentes en Bayer. Sin debayer '
            'y sin RViz. Graba con: ros2 bag record -a  (o record_dataset.sh)')

    return LaunchDescription(
        args + [validate_mode(), cameras] + camera_tfs() +
        [processing, recording_note])
