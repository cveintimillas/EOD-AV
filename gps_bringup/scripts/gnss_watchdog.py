#!/usr/bin/env python3
"""Vigila /fix y dice EN VOZ ALTA en cual de los tres estados esta el GNSS.

Sin esto los tres se parecen: RViz vacio y ningun error.

  1. NO LLEGA /fix          -> problema de software/gpsd. El nodo gpsd_client no
                               cargo, o gpsd no tiene el device abierto (mientras
                               online==0 gpsd_client descarta todo antes de
                               publicar).
  2. LLEGA /fix SIN FIX     -> la cadena ROS esta bien, falta senal. lat/lon en
                               NaN y status=-1. Es lo normal bajo techo.
  3. LLEGA /fix CON FIX     -> todo bien; recien aca aparece la TF map->base_link
                               y empieza a dibujarse /gps_path.

No publica NADA: es solo diagnostico por consola, asi que puede correr tambien
durante la grabacion sin ensuciar el bag (aparece en `ros2 node list` pero
`ros2 node info` no le lista ningun publisher salvo /rosout).

Parametros:
  fix_topic      (/fix)  topico a vigilar
  timeout_s      (5.0)   silencio tolerado antes de avisar que no llega nada
  report_every_s (30.0)  cada cuanto repetir el estado mientras no cambie
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import NavSatFix

NO_DATA, NO_FIX, FIX = 'NO_DATA', 'NO_FIX', 'FIX'


class GnssWatchdog(Node):
    """Reporta por consola el estado del GNSS a partir de /fix."""

    def __init__(self):
        super().__init__('gnss_watchdog')

        fix_topic = self.declare_parameter('fix_topic', '/fix').value
        self._timeout = float(self.declare_parameter('timeout_s', 5.0).value)
        self._report_every = float(
            self.declare_parameter('report_every_s', 30.0).value)

        # BEST_EFFORT para matchear con cualquier publicador (gpsd_client
        # publica RELIABLE, pero un bag puede ofrecer otra cosa).
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._sub = self.create_subscription(
            NavSatFix, fix_topic, self._on_fix, qos)

        self._state = None
        self._count = 0
        self._last_msg_mono = None
        self._last_report_mono = None
        self._fix_topic = fix_topic

        # El reloj de pared (no el de ROS): esto mide "hace cuanto que no llega
        # un mensaje", que es tiempo real aunque se este reproduciendo un bag.
        self._timer = self.create_timer(1.0, self._tick)
        self.get_logger().info(
            'gnss_watchdog vigilando %s (timeout %.1f s)'
            % (fix_topic, self._timeout))

    # -- helpers ------------------------------------------------------------

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _announce(self, state, message, level='info'):
        """Loguea solo cuando cambia el estado, o cada report_every_s."""
        now = self._now()
        changed = state != self._state
        due = (self._last_report_mono is None
               or now - self._last_report_mono >= self._report_every)
        if not changed and not due:
            return
        self._state = state
        self._last_report_mono = now
        getattr(self.get_logger(), level)(message)

    # -- callbacks ----------------------------------------------------------

    def _on_fix(self, msg):
        self._count += 1
        self._last_msg_mono = self._now()

        invalid = math.isnan(msg.latitude) or math.isnan(msg.longitude)
        if invalid:
            self._announce(
                NO_FIX,
                'GNSS SIN FIX: llegan mensajes en %s (%d) pero lat/lon son NaN '
                '(status=%d). La cadena ROS esta BIEN -- falta senal. Bajo techo '
                'es lo esperado; la antena necesita cielo despejado y el primer '
                'lock en frio puede tardar varios minutos. Hasta que haya fix no '
                'hay TF map->base_link, y RViz va a decir que el frame "map" no '
                'existe.' % (self._fix_topic, self._count, msg.status.status),
                'warn')
        else:
            self._announce(
                FIX,
                'GNSS CON FIX: lat=%.7f lon=%.7f alt=%.1f status=%d (%d msgs). '
                'Ya deberia existir la TF map->base_link y dibujarse /gps_path.'
                % (msg.latitude, msg.longitude, msg.altitude,
                   msg.status.status, self._count),
                'info')

    def _tick(self):
        if self._last_msg_mono is not None and \
                self._now() - self._last_msg_mono < self._timeout:
            return
        seen = ('nunca llego un mensaje' if self._count == 0
                else 'ultimo mensaje hace %.0f s'
                     % (self._now() - self._last_msg_mono))
        self._announce(
            NO_DATA,
            'GNSS MUDO: %s en %s. Esto NO es falta de senal: sin lock igual '
            'deberian llegar mensajes con NaN. Revisa (a) que el componente '
            'gpsd_client haya cargado -- `ros2 node list | grep gpsd_client`, y '
            'si solo aparece el contenedor es que fallo al construirse; (b) que '
            'gpsd tenga el device abierto -- `gpspipe -w -n 5 localhost:2947`, '
            'porque mientras online==0 gpsd_client descarta todo antes de '
            'publicar. Diagnostico completo: `ros2 run gps_bringup '
            'diagnose_gnss.sh`.' % (seen, self._fix_topic),
            'error')


def main(args=None):
    rclpy.init(args=args)
    node = GnssWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
