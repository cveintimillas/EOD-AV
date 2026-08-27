"""Modo LIVE/PLAYBACK: procesar y visualizar, sin importar de donde vengan los datos.

Live y playback NO son dos modos: son este mismo pipeline con distinta fuente.

    live      ros2 launch eod_av_launch all_sensors.launch.py
              (los drivers publican y este pipeline procesa)

    playback  ros2 launch eod_av_launch processing.launch.py bag:=<bag>
              (el bag publica lo mismo y este pipeline procesa igual)

Lo que produce, crudo -> derivado:

    /unfiltered_radar_packet_1 --filtros/DBSCAN--> nubes + clusters del radar
    /fix                       --fix_to_path-----> /gps_path (trayectoria)
    .../image (bayer)          --debayer opcional-> .../image_color

Lo que NO necesita procesarse: la nube del LiDAR ya viene lista (/lidar_points),
porque el driver la publica directamente y se graba asi. Es un dato
autocontenido: no depende de ningun archivo de correccion externo para poder
leerse dentro de 2 anios.

COLOR DE LAS CAMARAS: RViz no demosaica Bayer, asi que por defecto las muestra
en BLANCO Y NEGRO. Es suficiente para verificar encuadre, foco y exposicion.
Para ver color hay dos caminos:
    - `rqt_image_view /arena_camera_node/enp6s0/image`  (cv_bridge si demosaica,
      cero nodos extra: el camino recomendado)
    - debayer:=true, que levanta un nodo por camara y apunta RViz a
      .../image_color (mas trafico local y mas piezas moviles)
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from eod_av_launch.bringup import flag, scoped
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAMERA_IFACES = ('enp6s0', 'enp7s0', 'enp8s0')


def _processing(context):
    # La fuente se resuelve primero porque de ella depende el reloj.
    bag = LaunchConfiguration('bag').perform(context).strip()

    # 'auto' (default): el reloj sale del bag cuando hay bag, y del sistema
    # cuando la fuente son los sensores. Antes esto era un booleano que habia
    # que acordarse de poner a mano junto con --clock; olvidarlo dejaba a los
    # nodos derivados y a RViz mirando un reloj distinto al de los datos.
    raw_sim_time = LaunchConfiguration('use_sim_time').perform(context).strip().lower()
    if raw_sim_time == 'auto':
        use_sim_time = bool(bag)
    else:
        use_sim_time = flag(context, 'use_sim_time')

    actions = []

    # ---- Camaras: debayer OPCIONAL (por defecto no) ---------------------
    debayer_on = flag(context, 'enable_cameras') and flag(context, 'debayer')
    if debayer_on:
        rate = float(LaunchConfiguration('debayer_rate_hz').perform(context))
        actions.append(LogInfo(
            msg='[processing] debayer ON -> .../image_color a %.1f Hz' % rate))
        for iface in CAMERA_IFACES:
            actions.append(Node(
                package='eod_av_launch', executable='debayer_node',
                name='debayer_' + iface,
                namespace='/arena_camera_node/' + iface,
                output='screen', emulate_tty=True,
                parameters=[{'use_sim_time': use_sim_time, 'max_rate_hz': rate}]))
    elif flag(context, 'enable_cameras'):
        actions.append(LogInfo(
            msg='[processing] camaras en Bayer: RViz las muestra en BLANCO Y '
                'NEGRO (no demosaica). Para color: '
                'rqt_image_view /arena_camera_node/enp6s0/image  '
                '(o relanzar con debayer:=true)'))

    # ---- Radar: detecciones crudas -> filtrado, nubes, clusters --------
    if flag(context, 'enable_radar'):
        # scoped(): mismo motivo que en el resto del paquete -- los
        # launch_arguments de un include se escriben en el scope actual y no se
        # revierten. Aca no rompia nada por el orden de evaluacion, pero es la
        # regla del proyecto y es exactamente el patron que causo el bug v5.1.
        actions.append(scoped(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ars430_ros_publisher'),
                             'launch', 'radar_live.launch.py')),
            launch_arguments={
                'rviz': 'false',
                'capture': 'false',   # la fuente es el topico, no la red
                'processing': 'true',
                'publish_tf': 'false',
                'frame_id': 'radar_fixed',
            }.items())))

    # ---- GNSS: /fix -> pose (TF) y trayectoria (Path) -------------------
    # ESTE es el unico sitio donde corre fix_to_path, tanto en vivo como
    # reproduciendo un bag: mismo nodo, mismo nombre, mismos topicos. Por eso el
    # grafo de procesamiento es identico en los dos casos.
    # Es OPCIONAL (fix_to_path:=false lo apaga) porque todo lo que produce
    # -- la TF map->base_link y /gps_path -- se deriva de /fix.
    if flag(context, 'enable_gnss') and flag(context, 'fix_to_path'):
        actions.append(Node(
            package='gps_bringup', executable='fix_to_path.py',
            name='fix_to_path', output='screen',
            parameters=[{
                'frame_id': 'map',
                'fix_topic': '/fix',
                'child_frame': 'base_link',
                # TF siempre; el Path solo si se pide (es lo caro).
                'publish_path': flag(context, 'publish_path'),
                # Filtro de movimiento del Path: un receptor quieto tiene ruido
                # y sin esto dibuja una maranha de metros sin haberse movido.
                # Filtra la VISTA, no el dato: /fix se graba entero.
                'min_distance_m': float(
                    LaunchConfiguration('path_min_distance').perform(context)),
                'smooth_window': int(
                    LaunchConfiguration('path_smooth_window').perform(context)),
                'origin_settle_fixes': int(
                    LaunchConfiguration('origin_settle_fixes').perform(context)),
                'use_sim_time': use_sim_time,
            }]))

    # ---- Fuente rosbag (opcional): lo UNICO que distingue playback de live
    if bag:
        cmd = ['ros2', 'bag', 'play', bag]
        if use_sim_time:
            # --clock publica /clock; sin esto los nodos con use_sim_time
            # quedan clavados en t=0 y no procesan nada.
            cmd.append('--clock')
        actions.append(LogInfo(
            msg='[processing] fuente = bag %s (use_sim_time=%s)'
                % (bag, use_sim_time)))
        actions.append(ExecuteProcess(cmd=cmd, output='screen'))
    else:
        actions.append(LogInfo(msg='[processing] fuente = topicos en vivo'))

    # ---- RViz -----------------------------------------------------------
    if flag(context, 'rviz'):
        config = os.path.join(
            get_package_share_directory('eod_av_launch'), 'rviz',
            LaunchConfiguration('rviz_config').perform(context))

        # Las configs apuntan al topico CRUDO (.../image). Si el debayer esta
        # activo, se genera una copia que mira .../image_color.
        if debayer_on:
            with open(config) as fh:
                text = fh.read()
            if '/image' in text:
                config = os.path.join(tempfile.gettempdir(),
                                      'eodav_color_' + os.path.basename(config))
                with open(config, 'w') as fh:
                    fh.write(text.replace('/image\n', '/image_color\n'))

        actions.append(LogInfo(msg='[processing] RViz -> ' + config))
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            output='screen', arguments=['-d', config],
            parameters=[{'use_sim_time': use_sim_time}]))

    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'bag', default_value='',
            description='Ruta de un rosbag a reproducir. Vacio = fuente en vivo.'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config', default_value='all_sensors.rviz',
            description='Nombre del .rviz dentro de eod_av_launch/rviz/.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='auto',
            description='auto = true cuando hay bag:= (se reproduce con --clock), '
                        'false con sensores en vivo. true|false fuerza el valor.'),
        DeclareLaunchArgument(
            'debayer', default_value='false',
            description='true: nodo por camara que publica .../image_color. '
                        'Por defecto false; para ver color alcanza rqt_image_view.'),
        DeclareLaunchArgument('debayer_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('enable_cameras', default_value='true'),
        DeclareLaunchArgument('enable_radar', default_value='true'),
        DeclareLaunchArgument('enable_gnss', default_value='true'),
        DeclareLaunchArgument(
            'fix_to_path', default_value='true',
            description='Lanzar fix_to_path: TF map->base_link (pose del '
                        'vehiculo) a partir de /fix. Opcional: todo lo que '
                        'produce es derivado.'),
        DeclareLaunchArgument(
            'publish_path', default_value='true',
            description='Ademas de la TF, publicar /gps_path (trayectoria). '
                        'Es la parte cara: acumula y republica el historial.'),
        DeclareLaunchArgument(
            'path_min_distance', default_value='0.2',
            description='Piso del umbral de movimiento del Path, en metros. El '
                        'umbral real se adapta al ruido que reporta el receptor '
                        '(o a 1 m si no lo reporta). 0 dibuja todos los fixes, '
                        'ruido incluido.'),
        DeclareLaunchArgument(
            'path_smooth_window', default_value='10',
            description='Fixes promediados para decidir si hubo movimiento (y '
                        'para ubicar el punto). 1 desactiva el promediado.'),
        DeclareLaunchArgument(
            'origin_settle_fixes', default_value='20',
            description='Fixes promediados para fijar el origen del frame map. '
                        'El primer fix suele ser el peor: usarlo solo deja el '
                        'origen desplazado y arrastra el error toda la sesion.'),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_processing)])
