import os

from ament_index_python.packages import (PackageNotFoundError,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def _require_gpsd_client():
    """Falle temprano y claro si el paquete gpsd_client no esta instalado.

    Sin esto el unico sintoma es que el component_container escupe
    "Could not find requested resource in ament index" en medio del log y
    sigue corriendo vacio: no hay nodo gpsd_client, no hay /fix, y el mensaje
    no dice ni que paquete falta ni como instalarlo.

    gpsd_client se instala aparte (`ros-<distro>-gpsd-client`); ya esta
    declarado como exec_depend en el package.xml de gps_bringup, asi que
    `rosdep install --from-paths src --ignore-src -r -y` tambien lo trae.
    """
    try:
        get_package_share_directory('gpsd_client')
    except PackageNotFoundError:
        distro = os.environ.get('ROS_DISTRO', 'jazzy')
        raise RuntimeError(
            'El paquete gpsd_client no esta instalado, asi que no hay nada que '
            'publique /fix.\n'
            '  sudo apt install ros-%s-gpsd-client\n'
            '  # o, desde el workspace:\n'
            '  rosdep install --from-paths src --ignore-src -r -y\n'
            'Verificacion: ros2 component types | grep GPSDClientComponent'
            % distro)


def generate_launch_description():
    _require_gpsd_client()

    gpsd_host_arg = DeclareLaunchArgument(
        'gpsd_host', default_value='localhost',
        description='Host running gpsd (gpsd already owns the serial port, '
                    'see setup/setup_ptp_sync.sh)'
    )
    gpsd_port_arg = DeclareLaunchArgument(
        'gpsd_port', default_value='2947', description='gpsd TCP port'
    )
    frame_arg = DeclareLaunchArgument(
        'frame_id', default_value='gps_link',
        description='TF frame id for GPS')
    # Frecuencia con la que gpsd_client publica /fix. Es la frecuencia de
    # PUBLICACION; tiene que igualar (o superar) la del receptor.
    #
    # ESTE RECEPTOR VA A 10 Hz. Medido en el cable con `gpspipe -r`:
    #
    #   $GNGGA,174522.60,...   $GNGGA,174522.70,...   $GNGGA,174522.80,...
    #   $GNGGA,174522.90,...   $GNGGA,174523.00,...   $GNGGA,174523.10,...
    #
    # Una epoca cada 0,1 s, con dos decimales, clavadas en el borde de decima.
    # Y la posicion CAMBIA en cada una, asi que son diez fixes distintos por
    # segundo, no uno repetido.
    #
    # Por eso publish_rate:=1 esta MAL para este equipo: tira nueve de cada diez
    # posiciones reales. Para un dataset de vehiculo eso es perdida de dato.
    # (Un comentario anterior de este archivo decia que con 10 "se republica el
    # mismo fix diez veces": era falso, se escribio suponiendo un receptor de
    # 1 Hz sin haber mirado el NMEA.)
    #
    # Ojo con el reves: gpsd_client hace `while (waiting(0)) p = read();` y se
    # queda con la ULTIMA lectura, asi que si en un periodo entra mas de una
    # epoca, las del medio se PIERDEN sin aviso. Con publish_rate == frecuencia
    # del receptor el margen es justo; subirlo por encima no cuesta nada y da
    # aire.
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate', default_value='10',
        description='Hz a los que gpsd_client publica /fix. Tiene que igualar o '
                    'superar la frecuencia del receptor (aca 10 Hz): por debajo '
                    'se descartan fixes reales.')
    # De DONDE sale el header.stamp de /fix. Es la unica palanca que tenemos
    # sobre el sello del GNSS, y las dos ramas estan en gpsd_client:
    #
    #   gpsd_parser_base.cpp:193-201   parseNavSatFix -> /fix
    #     true  -> rclcpp::Time(data.fix.time)   campo de tiempo del receptor
    #     false -> fallback_stamp                el now() del host
    #
    #   gpsd_parser_base.cpp:106-107   parseGpsFix -> /extended_fix
    #     siempre el now() del host: parseGpsFix ni mira use_gps_time.
    #     Por eso /extended_fix da ~320 us y no sirve para juzgar el receptor.
    #
    # DEJARLO EN true. El campo del receptor es bueno: se verifico en el cable
    # (ver el comentario de publish_rate) que las epocas caen clavadas en el
    # borde de decima, .60 .70 .80 .90 .00, con dos decimales. La resolucion de
    # 100 ms no es un defecto: es exactamente lo que produce un receptor de
    # 10 Hz, y el PPS de este mismo equipo entra a .000008, asi que esta
    # enganchado a UTC.
    #
    # Que NO se concluya de una tabla de sync_check: con true, `now - stamp` da
    # cientos de ms. Eso es LATENCIA DE ENTREGA (epoca -> NMEA por serie ->
    # gpsd -> socket -> nodo), no error de sello. El sello sigue apuntando al
    # instante real de la solucion, que es lo unico que importa en un bag.
    #
    # Con false el numero de sync_check baja a ~150 us, y eso ENGANIA: el sello
    # pasa a ser la hora de llegada, o sea epoca + esa misma latencia. El error
    # no desaparece, se mete adentro del dato y deja de ser visible. Para un
    # dataset es estrictamente peor.
    #
    # false solo sirve como control de diagnostico: aisla el transporte de ROS
    # del receptor. Para eso ya esta /extended_fix, que usa el reloj del host
    # SIEMPRE (parseGpsFix ni mira este parametro) -- por eso da ~200 us y por
    # eso no sirve para juzgar al receptor.
    use_gps_time_arg = DeclareLaunchArgument(
        'use_gps_time', default_value='true',
        description='true (recomendado): header.stamp de /fix = epoca GNSS real '
                    'del receptor. false: = hora de llegada al host, que esconde '
                    'la latencia de entrega adentro del dato. No afecta a '
                    '/extended_fix, que siempre usa el reloj del host.')
    path_child_frame_arg = DeclareLaunchArgument(
        'path_child_frame',
        default_value='base_link',
        description="Frame that fix_to_path moves to the last fix (map -> this frame). "
                    "Defaults to 'base_link' so the whole static TF tree "
                    "(base_link -> lidar/radar/cameras, see "
                    "eod_av_launch/bringup.py) hangs off the GPS as "
                    "one rigid body. Was 'gps_link' before, which left base_link (and "
                    "thus every sensor frame) disconnected from map."
    )

    # um982_driver adds velocity + dual-antenna heading, which gpsd_client can't
    # parse (proprietary PVTSLNA/BESTNAVA/HPR). It does NOT replace gpsd_client:
    # /fix keeps coming from gpsd_client with PPS-disciplined timestamps for the
    # PTP pipeline (see setup/setup_ptp_sync.sh) -- um982_driver reads its own,
    # separate serial link (see its udev rule template) so it never contends
    # with gpsd for /dev/gps_pps. Default off until that second link and its
    # udev rule are confirmed on the bench; set enable_um982_heading:=true once
    # validated in the field.
    enable_um982_heading_arg = DeclareLaunchArgument(
        'enable_um982_heading', default_value='false',
        description='Launch um982_driver for /gnss/velocity + /gnss/heading '
                    '(needs its own dedicated serial link to the UM982, '
                    'separate from gpsd -- see '
                    'um982_driver/udev/99-eodav-um982-heading.rules.template).'
    )
    um982_heading_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('um982_driver'), 'launch', 'um982_heading.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('enable_um982_heading')),
        launch_arguments={'frame_id': LaunchConfiguration('frame_id')}.items(),
    )

    # SEGUNDA fuente para los MISMOS dos topicos, sin hardware extra.
    #
    # El receptor ya emite $GNHPR (rumbo, cabeceo) a ~10 Hz por el puerto que
    # gpsd tiene abierto -- verificado con `gpspipe -r`:
    #     $GNHPR,171035.80,202.4681,000.4902,000.0000,5,10,0.00,0999*5D
    # gpsd no la traduce a nada que ROS consuma, pero multiplexa por TCP, asi
    # que un cliente mas no pelea por el tty. Este nodo la lee de ahi y saca
    # /gnss/heading; /gnss/velocity lo arma de /extended_fix, que ya trae
    # speed/track/climb parseados por gpsd.
    #
    # Frente a enable_um982_heading: no necesita un segundo adaptador, ni la
    # regla udev, ni la libreria um982, y ademas sella con la hora UTC que trae
    # la propia sentencia en vez de la hora de lectura. Por eso este es el
    # camino por defecto para tener rumbo.
    #
    # LOS DOS PUBLICAN LO MISMO: prender ambos deja dos nodos peleando por
    # /gnss/heading. Elegir uno.
    enable_gnss_heading_arg = DeclareLaunchArgument(
        'enable_gnss_heading', default_value='true',
        description='Publicar /gnss/heading y /gnss/velocity leyendo las '
                    '$GNHPR que ya pasan por gpsd. No necesita hardware extra. '
                    'Excluyente con enable_um982_heading.'
    )
    gnss_heading_gpsd_node = Node(
        package='um982_driver',
        executable='gnss_heading_gpsd',
        name='gnss_heading_gpsd',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_gnss_heading')),
        parameters=[{
            'gpsd_host': LaunchConfiguration('gpsd_host'),
            'gpsd_port': ParameterValue(
                LaunchConfiguration('gpsd_port'), value_type=int),
            'frame_id': LaunchConfiguration('frame_id'),
            'use_gps_time': ParameterValue(
                LaunchConfiguration('use_gps_time'), value_type=bool),
        }],
    )

    # gpsd (levantado por setup/setup_ptp_sync.sh) ya tiene abierto el puerto
    # serie del simpleRTK3B para alimentar el refclock NMEA+PPS de chrony. Este
    # nodo se conecta a gpsd por TCP en vez de reabrir el ttyUSB directamente,
    # evitando que dos procesos lean el mismo puerto serie a la vez.
    gpsd_client_container = ComposableNodeContainer(
        name='gpsd_client_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gpsd_client',
                plugin='gpsd_client::GPSDClientComponent',
                name='gpsd_client',
                # LOS 7 PARAMETROS SON OBLIGATORIOS. GPSDClientComponent los
                # declara con la sobrecarga SOLO-TIPO de rclcpp:
                #
                #   this->declare_parameter("override_augmentation_source",
                #                           rclcpp::PARAMETER_BOOL);
                #
                # Esa sobrecarga declara el parametro SIN valor por defecto, o
                # sea que exige un override: si falta aunque sea uno,
                # declare_parameter lanza NoParameterOverrideProvidedException,
                # el constructor del componente revienta y el
                # component_container NO carga el componente. El contenedor
                # sigue vivo, asi que a simple vista "el launch arranco bien",
                # pero no hay nodo gpsd_client y por lo tanto no existen ni
                # /fix ni /extended_fix.
                #
                # Aca faltaba override_augmentation_source. Los otros seis ya
                # estaban, y por eso el sintoma era exactamente ese: launch sin
                # errores visibles y /fix mudo.
                #
                # Los valores repiten los defaults internos del nodo (los
                # miembros que usa get_parameter_or), asi que esto no cambia
                # ningun comportamiento: solo satisface el requisito de que
                # existan.
                parameters=[{
                    'host': LaunchConfiguration('gpsd_host'),
                    'port': LaunchConfiguration('gpsd_port'),
                    'frame_id': LaunchConfiguration('frame_id'),
                    'publish_rate': ParameterValue(
                        LaunchConfiguration('publish_rate'), value_type=int),
                    # De donde sale el header.stamp de /fix. Ver el comentario
                    # largo en use_gps_time_arg: el campo del receptor llega
                    # cuantizado a 100 ms y con la fase paseandose, asi que
                    # esto es una palanca real, no cosmetica.
                    'use_gps_time': ParameterValue(
                        LaunchConfiguration('use_gps_time'), value_type=bool),
                    # false = publicar /fix SIEMPRE que el receptor este
                    # online, tenga o no lock. Es deliberado: con true,
                    # parseNavSatFix() devuelve nullopt cuando la varianza no
                    # es valida y /fix queda mudo sin fix, que es justo cuando
                    # mas hace falta ver que el receptor responde.
                    'check_fix_by_variance': False,
                    # Sin uso en este stack (RTK propio, no augmentation
                    # externa). Se pasa porque es obligatorio, no porque haga
                    # falta: false es el default interno del nodo.
                    'override_augmentation_source': False,
                }],
            ),
        ],
        output='screen',
    )

    # El watchdog corre en LOS DOS MODOS a proposito. No publica nada (solo
    # loguea), asi que no ensucia el bag, y durante una grabacion larga saber
    # que el GNSS perdio el lock vale mas que el ruido en `ros2 node list`.
    watchdog_arg = DeclareLaunchArgument(
        'gnss_watchdog', default_value='true',
        description='Nodo que avisa por consola si /fix no llega, llega sin fix, '
                    'o llega con fix. No publica ningun topico.')
    gnss_watchdog_node = Node(
        package='gps_bringup',
        executable='gnss_watchdog.py',
        name='gnss_watchdog',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gnss_watchdog')),
    )

    # fix_to_path NO se lanza por defecto: este launch es una FUENTE de datos
    # (publica /fix y /extended_fix, que es lo que sale del receptor). Tanto la
    # TF map -> base_link como /gps_path son DERIVADOS de /fix, asi que son
    # procesamiento y no deben correr durante la adquisicion: se reconstruyen
    # despues, en el modo live/playback, con el mismo nodo.
    #
    # En el flujo de eod_av_launch quien lanza fix_to_path es SIEMPRE
    # processing.launch.py, tanto en vivo como reproduciendo un bag, para que la
    # cadena de procesamiento sea identica en ambos casos.
    #
    # enable_path:=true queda disponible solo para usar este paquete por su
    # cuenta, fuera de eod_av_launch. No lo actives junto a processing: habria
    # dos nodos publicando la misma TF.
    enable_path_arg = DeclareLaunchArgument(
        'enable_path', default_value='false',
        description='Lanzar fix_to_path aqui (TF map->base_link y /gps_path). '
                    'false por defecto: en eod_av_launch lo lanza processing.'
    )
    publish_path_arg = DeclareLaunchArgument(
        'publish_path', default_value='true',
        description='Si enable_path:=true, publicar tambien /gps_path ademas '
                    'de la TF.'
    )
    fix_to_path_node = Node(
        package='gps_bringup',
        executable='fix_to_path.py',
        name='fix_to_path',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_path')),
        parameters=[{
            'frame_id': 'map',
            'fix_topic': '/fix',
            'child_frame': LaunchConfiguration('path_child_frame'),
            'publish_path': ParameterValue(
                LaunchConfiguration('publish_path'), value_type=bool),
        }]
    )

    return LaunchDescription([
        gpsd_host_arg,
        gpsd_port_arg,
        frame_arg,
        publish_rate_arg,
        use_gps_time_arg,
        path_child_frame_arg,
        enable_path_arg,
        publish_path_arg,
        enable_um982_heading_arg,
        enable_gnss_heading_arg,
        watchdog_arg,
        gpsd_client_container,
        gnss_watchdog_node,
        fix_to_path_node,
        um982_heading_launch,
        gnss_heading_gpsd_node,
    ])
