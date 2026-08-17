"""Helpers compartidos por los launch de eod_av_launch.

Vive aca (modulo Python instalado del paquete) y no en un launch aparte, para
que en launch/ existan EXACTAMENTE los 5 bring-up (cameras / lidar / radar /
gnss / all_sensors) mas el pipeline (processing).

Contiene:
  - static_tf() y los helpers por sensor (lidar_tf, radar_tf, camera_tfs,
    gnss_tf): el arbol TF estatico base_link -> <sensor>
  - resolve_mode() / validate_mode() / if_live() / if_record(): los DOS modos
  - scoped(): aislamiento de los IncludeLaunchDescription
  - rviz(): nodo rviz2 condicionado a rviz:=true/false, con LogInfo que
    imprime la config usada (asi se ve en consola si RViz se activo o no)
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import GroupAction, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def scoped(*actions, condition=None):
    """Aisla un IncludeLaunchDescription para que sus launch_arguments NO se filtren.

    IMPRESCINDIBLE: IncludeLaunchDescription aplica cada launch_argument como un
    SetLaunchConfiguration en el scope ACTUAL y no lo revierte al terminar. Sin
    este wrapper, incluir un sub-launch con {'rviz': 'false'} dejaba rviz='false'
    en ESTE launch, y el nodo rviz2 declarado despues del include no arrancaba
    nunca (sintoma exacto que se vio en radar.launch.py y all_sensors.launch.py).
    GroupAction(scoped=True) hace push/pop de las configuraciones.
    """
    return GroupAction(list(actions), scoped=True, forwarding=True,
                       condition=condition)


BASE_FRAME = 'base_link'

# ---------------------------------------------------------------------------
# MODOS DE OPERACION -- son EXACTAMENTE DOS
#
#   live      ver los datos: todo el procesamiento derivado encendido. La
#             fuente puede ser los sensores (bring-up) o un bag
#             (processing.launch.py bag:=<ruta>). Playback NO es un tercer
#             modo: es este mismo modo con otra fuente, y por eso el grafo de
#             nodos derivados es identico en los dos casos.
#
#   record    grabar el dataset. SOLO datos crudos: sin RViz, sin nubes
#             derivadas, sin trayectoria, sin filtros, sin debayer. Todo lo
#             que se apaga aca es reconstruible despues a partir del crudo,
#             asi que no se pierde informacion -- se gana ancho de banda y
#             regularidad.
#
# Medido en banco, record baja el caudal de ~400 MB/s a ~127 MB/s (3,1x).
#
# Hubo un MODE_PLAYBACK como tercer modo (etapa v6). Se elimino al unificar
# live+playback: seguia declarado en MODES aunque nadie lo distinguia, asi que
# `mode:=playback` pasaba la validacion y terminaba levantando los sensores
# fisicos -- justo lo contrario de lo que el nombre promete.
# ---------------------------------------------------------------------------
MODE_LIVE = 'live'
MODE_RECORD = 'record'
MODES = (MODE_LIVE, MODE_RECORD)


def resolve_mode(context, arg_name='mode'):
    """Lee y valida el argumento `mode`, devolviendo la cadena normalizada."""
    mode = LaunchConfiguration(arg_name).perform(context).strip().lower()
    if mode not in MODES:
        raise RuntimeError(
            "%s:=%r invalido. Valores validos: %s" % (arg_name, mode, ', '.join(MODES)))
    return mode


def validate_mode(arg_name='mode'):
    """Accion que aborta el launch temprano si `mode` no es live|record.

    Los bring-up deciden con `if_live()`/`if_record()`, que son condiciones y
    no validan: un `mode:=recrod` mal tipeado no matchearia ninguna y el launch
    arrancaria a medias sin decir por que. Esto lo corta en el arranque.
    """
    def _check(context, *_args, **_kwargs):
        resolve_mode(context, arg_name)
        return []
    return OpaqueFunction(function=_check)


def if_live(arg_name='mode'):
    """Condicion: verdadera con mode:=live (fuentes + procesamiento)."""
    return IfCondition(
        PythonExpression(["'", LaunchConfiguration(arg_name), "' == '", MODE_LIVE, "'"]))


def if_record(arg_name='mode'):
    """Condicion: verdadera con mode:=record (solo fuentes crudas)."""
    return IfCondition(
        PythonExpression(["'", LaunchConfiguration(arg_name), "' == '", MODE_RECORD, "'"]))


def flag(context, name):
    """Evalua un argumento booleano de launch como bool de Python."""
    return LaunchConfiguration(name).perform(context).strip().lower() in ('true', '1')


# ---------------------------------------------------------------------------
# EXTRINSECAS (base_link -> sensor)
#
# *** SIN MEDIR: traslaciones en 0 a proposito ***  Rellenar x/y/z reales
# (metros, cinta metrica/CAD respecto al origen de base_link) antes de tratar
# una grabacion como calibrada. Un 0 evidente falla ruidosamente (los frames se
# superponen en el origen); un numero "plausible" inventado corrompe la fusion
# en silencio.
#
# La rotacion del radar tambien arranca en 0: ver RADAR_YAW_DEG abajo.
# ---------------------------------------------------------------------------

# TODOS los sensores van SIN ROTACION (identidad): cada frame queda con la
# orientacion por defecto que muestra RViz (+X adelante, +Y izquierda, +Z arriba,
# REP-103). Ningun sensor recibe trato especial.
#
# El radar tampoco: RADAR_YAW_DEG = 0. Cuando toque calibrar el montaje real, el
# angulo se busca en caliente y despues se escribe aca:
#   ros2 launch eod_av_launch all_sensors.launch.py radar_tf_live:=true
#   ros2 param set /radar_tf yaw_deg <grados>

RADAR_YAW_DEG = 0.0

# Nombres de frame (deben coincidir con lo que publica cada driver)
LIDAR_FRAME = 'hesai_lidar'
RADAR_FRAME = 'radar_fixed'
CAMERA_FRAMES = ('camera_enp6s0', 'camera_enp7s0', 'camera_enp8s0')
# gpsd_client estampa /fix con frame_id gps_link (ver gps_bringup/launch/
# gps.launch.py). Sin esta TF ese frame no existe en el arbol: cualquier
# consumidor que quiera pasar el fix a base_link falla con "Missing transform
# from frame gps_link". OJO: es gps_link el que cuelga de base_link, y NO al
# reves -- la TF dinamica de fix_to_path va map -> base_link, y un frame no
# puede tener dos padres.
GNSS_FRAME = 'gps_link'

# UNA SOLA REFERENCIA COMUN: todos los sensores cuelgan directamente de
# base_link, cada uno con su propia TF independiente:
#
#     map --(GNSS, dinamica)--> base_link --> hesai_lidar
#                                         |-> radar_fixed
#                                         |-> camera_enp6s0 / 7s0 / 8s0
#                                         |-> gps_link
#
# base_link es el unico punto de union, y map -> base_link (que publica
# fix_to_path con el GNSS) mueve TODO el conjunto como un rigido mientras se
# traza la trayectoria. Colgar el radar del LiDAR lo dejaba huerfano si el LiDAR
# no corria (enable_lidar:=false); para atarlo al LiDAR igualmente:
#   ros2 launch eod_av_launch radar.launch.py radar_parent:=hesai_lidar
RADAR_PARENT_FRAME = BASE_FRAME


def static_tf(name, child_frame, x=0.0, y=0.0, z=0.0,
              roll=0.0, pitch=0.0, yaw=0.0, parent=BASE_FRAME):
    """TF estatica parent -> child_frame. Angulos en RADIANES."""
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name=name,
        arguments=[
            '--x', str(x), '--y', str(y), '--z', str(z),
            '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
            '--frame-id', parent, '--child-frame-id', child_frame,
        ])


def lidar_tf():
    return static_tf('static_tf_hesai_lidar', LIDAR_FRAME)


def radar_tf(yaw_rad, parent=RADAR_PARENT_FRAME):
    """TF ESTATICA del radar, colgada del LiDAR por defecto (RADAR_PARENT_FRAME).

    yaw_rad alinea el frente del radar con el del padre (ver RADAR_YAW_DEG).
    """
    return static_tf('static_tf_radar_fixed', RADAR_FRAME,
                     yaw=yaw_rad, parent=parent)


def radar_tf_tunable(yaw_deg, parent=RADAR_PARENT_FRAME):
    """TF del radar AJUSTABLE EN CALIENTE (nodo tf_tuner, ver tf_tuner.py).

    Permite girar/desplazar el radar sin reiniciar, mirando RViz en vivo:

        ros2 param set /radar_tf yaw_deg -45.0
        ros2 param set /radar_tf x 0.35

    Cuando el valor sea el correcto, copialo a RADAR_YAW_DEG (arriba) y volve a
    la TF estatica con radar_tf_live:=false para grabar.
    """
    return Node(
        package='eod_av_launch', executable='tf_tuner', name='radar_tf',
        output='screen',
        parameters=[{
            'parent_frame': parent,
            'child_frame': RADAR_FRAME,
            'yaw_deg': float(yaw_deg),
        }])


def camera_tfs():
    return [static_tf('static_tf_' + f, f) for f in CAMERA_FRAMES]


def gnss_tf():
    """TF estatica base_link -> gps_link (la antena, sin medir todavia)."""
    return static_tf('static_tf_' + GNSS_FRAME, GNSS_FRAME)


def rviz(config_basename, arg_name='rviz'):
    """Nodo rviz2 + logs. Devuelve una lista de acciones para el LaunchDescription.

    Usa la config eod_av_launch/rviz/<config_basename>. El LogInfo imprime la
    ruta exacta en consola cuando rviz:=true (y avisa cuando esta apagado), de
    modo que siempre se ve si la visualizacion se activo.
    """
    config = os.path.join(
        get_package_share_directory('eod_av_launch'), 'rviz', config_basename)
    enabled = IfCondition(LaunchConfiguration(arg_name))

    actions = [
        LogInfo(condition=enabled, msg=['[eod_av_launch] RViz ON -> ', config]),
        LogInfo(condition=UnlessCondition(LaunchConfiguration(arg_name)),
                msg='[eod_av_launch] RViz OFF (rviz:=true para abrirlo)'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             output='screen',            # que se vea cualquier error de RViz
             arguments=['-d', config],
             condition=enabled),
    ]
    if not os.path.exists(config):
        actions.insert(0, LogInfo(
            msg='[eod_av_launch] AVISO: no existe ' + config +
                ' -- reconstrui el paquete (colcon build --packages-select '
                'eod_av_launch) para instalar rviz/'))
    return actions
