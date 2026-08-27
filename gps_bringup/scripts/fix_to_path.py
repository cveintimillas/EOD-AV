#!/usr/bin/env python3
"""Convierte sensor_msgs/NavSatFix (/fix) en TF dinamica y trayectoria.

Salidas:
  - TF map -> child_frame      -> pose actual del vehiculo. En eod_av_launch
                                  child_frame es base_link, que es el unico
                                  hijo directo de map y de quien cuelgan las
                                  extrinsecas de todos los sensores.
  - nav_msgs/Path (/gps_path)  -> trayectoria en metros (frame 'map', ENU
                                  local). Opcional: publish_path:=false.

Nodo DERIVADO: corre solo en eod_av_launch/launch/processing.launch.py, o sea
en modo live (con sensores o con un bag), nunca en mode:=record. Todo lo que
publica se reconstruye despues desde /fix, que es lo que se graba.
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

R = 6378137.0  # radio ecuatorial WGS84 (m)


class FixToPath(Node):
    def __init__(self):
        super().__init__('fix_to_path')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('fix_topic', '/fix')
        self.declare_parameter('child_frame', 'gps_link')
        # Cap del historial del Path: sin tope, self.path.poses crece sin limite
        # y se republica ENTERO en cada fix (10 Hz) -> memoria no acotada y
        # ancho de banda O(n^2) en grabaciones largas. 0 = sin limite (comportamiento
        # anterior); >0 = ventana deslizante de los ultimos N puntos.
        self.declare_parameter('max_path_len', 10000)
        # Este nodo hace DOS cosas de costo muy distinto:
        #   - la TF map -> child_frame: un mensaje chico por fix, costo constante.
        #   - el Path /gps_path: ACUMULA poses y republica el historial entero en
        #     cada fix, asi que su costo crece con el tiempo transcurrido
        #     (~50 GB a lo largo de una hora).
        # publish_path:=false deja solo la TF: es lo que se usa al GRABAR, para
        # que el bag lleve la pose del vehiculo sin pagar el crecimiento O(n^2).
        # La trayectoria se reconstruye despues en el modo live/playback.
        self.declare_parameter('publish_path', True)

        # --- Filtro de MOVIMIENTO del Path (solo visualizacion) -------------
        # Un receptor quieto NO da la misma posicion dos veces: cada fix trae
        # ruido, asi que acumular todos dibuja una maranha de varios metros sin
        # que nada se haya movido. No es un error del software; es el ruido del
        # GNSS. Solo se agrega un punto al Path si el vehiculo se desplazo mas
        # que un umbral respecto del ultimo punto dibujado.
        #
        # IMPORTANTE: esto filtra la VISTA, no el dato. /fix se graba entero,
        # con todo su ruido, que es lo correcto para el dataset: el Path es
        # derivado y se reconstruye cuando se quiera, con otro umbral si hace
        # falta.
        # Piso absoluto, en metros. Se mantiene chico a proposito: el trabajo
        # pesado lo hace el termino adaptativo de abajo. Un piso grande
        # arruinaria una trayectoria RTK, donde el ruido es de centimetros y
        # los detalles de 30 cm son reales.
        self.declare_parameter('min_distance_m', 0.2)
        # El umbral real se adapta a la incertidumbre del receptor:
        #     umbral = max(min_distance_m, distance_sigma_k * sigma_horizontal)
        # Dos fixes consecutivos de un receptor QUIETO con ruido sigma estan
        # separados tipicamente ~1,4*sigma (dos muestras independientes), asi
        # que con k=2 el umbral queda por encima de casi todo el ruido y por
        # debajo de cualquier desplazamiento real. Sin RTK sigma es de metros y
        # el umbral sube solo; con RTK fijo baja a centimetros y la trayectoria
        # fina se conserva. 0 desactiva la parte adaptativa.
        self.declare_parameter('distance_sigma_k', 2.0)
        # sigma a usar cuando el receptor NO reporta covarianza
        # (position_covariance_type == 0). Sin esto el filtro se cae al piso de
        # 0,2 m y practicamente no filtra: medido con ruido de 1 m sigma, el
        # piso solo sacaba el 9 % de los puntos, mientras que el termino
        # adaptativo sacaba el 89 %.
        self.declare_parameter('fallback_sigma_m', 1.0)
        # Ventana de promediado para el Path. Comparar el fix crudo contra el
        # ultimo punto dibujado NO alcanza: la nube de ruido de un receptor
        # quieto es mas grande que el umbral, asi que en cuanto un fix cae en el
        # borde opuesto de la nube supera la distancia y se dibuja igual (medido:
        # 211 de 600 puntos seguian pasando el filtro). Promediando las ultimas
        # N muestras la nube se encoge por sqrt(N) y el gate recien funciona.
        # El punto se dibuja EN la posicion promediada, que ademas es la mejor
        # estimacion disponible. 0 o 1 desactiva el promediado.
        #
        # Cuesta medio segundo de retardo a 10 Hz: el Path va un poco atras de
        # la TF, que sigue siendo el fix crudo sin filtrar.
        self.declare_parameter('smooth_window', 10)
        # El PRIMER fix es sistematicamente el peor: el receptor todavia no
        # convergio. Fijar el origen con el ahi nomas deja el frame 'map'
        # desplazado decenas de metros respecto de donde realmente se arranco, y
        # como todo lo demas se mide contra el origen, el error se arrastra toda
        # la sesion. Se promedian los primeros N fixes validos: el error del
        # origen baja con la raiz de N y solo cuesta N/rate segundos de espera.
        self.declare_parameter('origin_settle_fixes', 20)

        self.frame = self.get_parameter('frame_id').value
        self.child = self.get_parameter('child_frame').value
        fix_topic = self.get_parameter('fix_topic').value
        self.max_path_len = self.get_parameter('max_path_len').value
        self.publish_path = self.get_parameter('publish_path').value
        self.min_distance_m = float(self.get_parameter('min_distance_m').value)
        self.distance_sigma_k = float(
            self.get_parameter('distance_sigma_k').value)
        self.fallback_sigma_m = float(
            self.get_parameter('fallback_sigma_m').value)
        self.smooth_window = max(1, int(self.get_parameter('smooth_window').value))
        self.origin_settle_fixes = max(
            1, int(self.get_parameter('origin_settle_fixes').value))

        self.origin = None            # (lat0, lon0) promediado, ver _settle
        self._settle = []             # fixes acumulados para promediar el origen
        self._nan_count = 0           # mensajes recibidos sin fix (lat/lon NaN)
        self._last_xy = None          # ultimo punto DIBUJADO en el Path
        self._window = deque(maxlen=self.smooth_window)   # ultimos (x, y)
        self.path = Path()
        self.path.header.frame_id = self.frame

        self.pub = self.create_publisher(Path, 'gps_path', 10) \
            if self.publish_path else None
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(NavSatFix, fix_topic, self.cb, 10)
        self.get_logger().info(
            f"Escuchando {fix_topic}: TF {self.frame} -> {self.child}"
            + (" + /gps_path" if self.publish_path else " (sin /gps_path)"))

    def _horizontal_sigma(self, msg: NavSatFix):
        """Desviacion horizontal (m) que reporta el receptor, o None."""
        # position_covariance_type == 0 es COVARIANCE_TYPE_UNKNOWN: el receptor
        # no dijo nada y la matriz no significa nada.
        if msg.position_covariance_type == 0:
            return None
        var = (msg.position_covariance[0] + msg.position_covariance[4]) / 2.0
        if not math.isfinite(var) or var <= 0.0:
            return None
        return math.sqrt(var)

    def _path_point(self, x, y, msg: NavSatFix):
        """Devuelva el (x, y) a dibujar, o None si no hubo movimiento real."""
        self._window.append((x, y))
        n = len(self._window)
        sx = sum(p[0] for p in self._window) / n
        sy = sum(p[1] for p in self._window) / n

        if self._last_xy is None:
            return (sx, sy)                  # el primer punto siempre entra

        threshold = self.min_distance_m
        if self.distance_sigma_k > 0.0:
            sigma = self._horizontal_sigma(msg)
            if sigma is None:
                # El receptor no reporta covarianza: se asume el sigma nominal
                # en vez de dejar el filtro sin su termino principal.
                sigma = self.fallback_sigma_m
            if sigma > 0.0:
                threshold = max(threshold, self.distance_sigma_k * sigma)

        moved = math.hypot(sx - self._last_xy[0], sy - self._last_xy[1])
        if moved < threshold:
            self.get_logger().debug(
                'quieto: %.2f m < umbral %.2f m, no se agrega punto'
                % (moved, threshold))
            return None
        return (sx, sy)

    def cb(self, msg: NavSatFix):
        # No usamos msg.status.status: gpsd_client siempre reporta STATUS_NO_FIX
        # con gpsd >= 3.23 en fixes GPS simples (gpsd omite el campo "status" en
        # su JSON salvo fixes DGPS o mejores, ver gitlab.com/gpsd/gpsd/-/issues/154).
        # lat/lon en NaN es la unica senal fiable de "sin fix" que da gpsd_client.
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            # Sin fix no hay pose que publicar, y por lo tanto no existe el
            # frame 'map'. RViz va a decir "Fixed Frame [map] does not exist"
            # hasta el primer fix valido. Se avisa (throttled) para que ese
            # error de RViz se pueda leer como "falta senal" y no como "esto
            # esta roto": son dos cosas muy distintas y se ven igual.
            self._nan_count += 1
            self.get_logger().warn(
                'Todavia sin fix: llegan mensajes en /fix (%d) pero lat/lon son '
                'NaN. No se publica TF %s -> %s ni /gps_path hasta el primer fix '
                'valido; RViz va a seguir diciendo que el frame "%s" no existe. '
                'La cadena ROS esta bien, falta senal del receptor.'
                % (self._nan_count, self.frame, self.child, self.frame),
                throttle_duration_sec=10.0)
            return

        if self.origin is None:
            self._settle.append((msg.latitude, msg.longitude))
            if len(self._settle) == 1:
                self.get_logger().info(
                    'PRIMER FIX VALIDO (lat=%.7f lon=%.7f). Promediando %d '
                    'fixes para fijar el origen; hasta entonces no se publica '
                    'la TF %s -> %s.'
                    % (msg.latitude, msg.longitude, self.origin_settle_fixes,
                       self.frame, self.child))
            if len(self._settle) < self.origin_settle_fixes:
                return
            n = len(self._settle)
            self.origin = (sum(p[0] for p in self._settle) / n,
                           sum(p[1] for p in self._settle) / n)
            self._settle = []
            self.get_logger().info(
                'ORIGEN FIJADO en lat=%.7f lon=%.7f (promedio de %d fixes). Ya '
                'se publica la TF %s -> %s%s, y el frame "%s" existe para RViz.'
                % (self.origin[0], self.origin[1], n, self.frame, self.child,
                   ' y /gps_path' if self.publish_path else '', self.frame))

        lat0, lon0 = self.origin
        # Proyeccion ENU local (equirectangular, suficiente para pocos km)
        x = math.radians(msg.longitude - lon0) * R * math.cos(math.radians(lat0))
        y = math.radians(msg.latitude - lat0) * R
        # msg.altitude se ignora a proposito: la trayectoria se publica plana
        # (z=0). La altura elipsoidal del GNSS es mucho mas ruidosa que la
        # planta y solo servia para que el Path flotara en RViz.

        # Se estampa con el tiempo del PROPIO fix, no con el reloj del nodo.
        # /fix viene con timestamp GPS/PPS (gpsd_client con use_gps_time), o
        # sea que es el mismo reloj disciplinado que usan las camaras y el
        # LiDAR: la pose queda alineada con el resto del dataset. Y ademas hace
        # que reproducir un bag de el resultado identico a la corrida en vivo,
        # sin depender de use_sim_time, de --clock ni del rate del playback.
        now = msg.header.stamp

        # --- Path (solo si publish_path; es la parte cara) ---
        # Se agrega un punto SOLO si hubo desplazamiento real. Quieto, el ruido
        # del receptor dibujaria una maranha de varios metros; ademas esto evita
        # republicar el historial entero 10 veces por segundo sin necesidad.
        point = self._path_point(x, y, msg) if self.publish_path else None
        if point is not None:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = self.frame
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self.path.header.stamp = now
            self.path.poses.append(pose)
            if self.max_path_len and len(self.path.poses) > self.max_path_len:
                # Ventana deslizante: conserva solo los ultimos max_path_len puntos.
                self.path.poses = self.path.poses[-self.max_path_len:]
            self.pub.publish(self.path)
            self._last_xy = point

        # --- TF map -> base_link (posicion actual; ver child_frame) ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.frame
        t.child_frame_id = self.child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = FixToPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
