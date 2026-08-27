"""Las 3 camaras LUCID Triton, cada una en su propia NIC.

FORMATO POR DEFECTO: bayer_rggb8, NO rgb8.
    Medido en banco: con rgb8 cada frame pesa 2048*1536*3 = 9.44 MB y la camara
    satura su enlace GigE (115 MB/s = 922 Mbps de ~950 utiles), lo que la
    limitaba a ~12 fps con intervalos irregulares (jitter de ~33 ms, hasta 190 ms).
    El sensor es Bayer: pedir rgb8 hace que el FIRMWARE interpole y mande 3
    bytes por pixel, o sea 3x los datos que realmente capturo.
    Con bayer_rggb8 son 3.15 MB/frame (~30% del enlace): mismo dato -- de hecho
    mas crudo -- con margen de sobra para que la entrega sea regular. El debayer
    se hace en postproceso (ver eod_av_launch/playback.launch.py).

    Si el patron Bayer nativo de tu Triton no fuera RGGB, cambialo con
    pixelformat:=bayer_bggr8 / bayer_gbrg8 / bayer_grbg8. Para volver al
    comportamiento anterior: pixelformat:=rgb8.

FRAME RATE: al liberar ancho de banda la camara subiria sola hacia ~36 fps y se
    comeria el margen recien ganado, asi que se fija con frame_rate (fps).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# (nombre del nodo / frame_id / topic, numero de serie de la camara)
# El sufijo enpXs0 es la NIC a la que esta cableada cada camara; la camara se
# identifica por SERIAL, no por interfaz.
CAMERAS = (
    ('enp6s0', '260500567'),
    ('enp7s0', '261203985'),
    ('enp8s0', '260500585'),
)


def _num(name, kind=float):
    return ParameterValue(LaunchConfiguration(name), value_type=kind)


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'pixelformat', default_value='bayer_rggb8',
            description='bayer_rggb8 (default, 1 B/px) | rgb8 (3 B/px, satura GigE).'),
        DeclareLaunchArgument(
            'frame_rate', default_value='12.0',
            description='Tope de fps (AcquisitionFrameRate). <=0 = sin tope.'),
        DeclareLaunchArgument(
            'width', default_value='2048'),
        DeclareLaunchArgument(
            'height', default_value='1536'),
        DeclareLaunchArgument(
            'gain_auto', default_value='Continuous',
            description='Off | Once | Continuous. Off usa el valor de gain.'),
        DeclareLaunchArgument('gain', default_value='13.0'),
        DeclareLaunchArgument(
            'exposure_auto', default_value='Continuous',
            description='Off | Once | Continuous. Off usa exposure_time (us).'),
        DeclareLaunchArgument('exposure_time', default_value='28000.0'),
        DeclareLaunchArgument('qos_reliability', default_value='reliable'),
        DeclareLaunchArgument('ptp_enable', default_value='true'),
    ]

    nodes = [
        Node(
            package='arena_camera_node',
            executable='start',
            name='arena_cam_' + iface,
            parameters=[{
                'serial': serial,
                'frame_id': 'camera_' + iface,
                'topic': '/arena_camera_node/' + iface + '/image',
                'qos_reliability': LaunchConfiguration('qos_reliability'),
                'pixelformat': LaunchConfiguration('pixelformat'),
                'width': _num('width', int),
                'height': _num('height', int),
                'frame_rate': _num('frame_rate'),
                'gain_auto': LaunchConfiguration('gain_auto'),
                'gain': _num('gain'),
                'exposure_auto': LaunchConfiguration('exposure_auto'),
                'exposure_time': _num('exposure_time'),
                'trigger_mode': False,
                'ptp_enable': _num('ptp_enable', bool),
            }])
        for iface, serial in CAMERAS
    ]

    return LaunchDescription(args + nodes)
