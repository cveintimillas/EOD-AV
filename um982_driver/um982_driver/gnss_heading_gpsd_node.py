"""Publica /gnss/heading y /gnss/velocity leyendo lo que YA pasa por gpsd.

POR QUE EXISTE ESTE NODO, HABIENDO um982_heading_node
`um982_heading_node` abre un SEGUNDO puerto serie dedicado al receptor y lee
los logs binarios de Unicore. Eso exige un adaptador aparte y una regla udev
que no puede apuntar al puerto de gpsd (dos procesos sobre el mismo tty ya
corrompio ambas lecturas una vez).

Este nodo no necesita nada de eso. El receptor YA emite el rumbo por el mismo
puerto que gpsd tiene abierto -- medido con `gpspipe -r`: ~10 Hz de $GNHPR y
$GNTHS mezcladas con GGA/RMC/GSV. gpsd multiplexa por TCP, asi que leerlas no
pelea con nadie. Cero hardware nuevo, cero riesgo sobre la cadena del PPS.

Los dos nodos publican los MISMOS topicos: no hay que correr los dos.

DE DONDE SALE CADA COSA
  /gnss/heading   <- $GNHPR por el socket de gpsd (rumbo y cabeceo).
                     gpsd NO traduce esta sentencia: gpsd_client publica
                     NavSatFix y GPSFix, y el `track` de GPSFix es rumbo sobre
                     el suelo, que parado no significa nada.
  /gnss/velocity  <- /extended_fix, que ya publica gpsd_client con speed,
                     track y climb ya parseados y con sus errores. Abrir una
                     segunda conexion para reparsear eso seria duplicar trabajo
                     que gpsd ya hizo.

TIMESTAMPS
$GNHPR trae su propia hora UTC, asi que /gnss/heading se sella con la EPOCA DEL
RECEPTOR, no con la hora de lectura (ver nmea_heading.epoch_desde_tod). Es el
mismo criterio que `use_gps_time` en /fix. /gnss/velocity hereda el header de
/extended_fix, que ya viene del reloj del host disciplinado por el PPS.
"""
import math
import socket
from typing import Optional

from geometry_msgs.msg import TwistWithCovarianceStamped

from gps_msgs.msg import GPSFix

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

from um982_driver.heading_node import euler_deg_to_quaternion
from um982_driver.nmea_heading import (epoch_desde_tod, parse_hpr,
                                       ths_invalida)

# sensor_msgs/Imu reserva -1 en el elemento 0 de una covarianza para decir "este
# campo no se estima". Se usa para la velocidad angular y la aceleracion, que
# este mensaje no lleva.
_NO_ESTIMADO = -1.0

# Varianza grande = "no confies en esto". Se usa para el ALABEO: con dos
# antenas hay una sola linea base y el giro alrededor de ella no es observable.
# El receptor manda 0.0000 igual, asi que sin esta marca el consumidor lo lee
# como un alabeo medido de cero grados.
_ALABEO_NO_OBSERVABLE = 1e6

# gps_msgs/GPSFix documenta los err_* como intervalos al 95 % de confianza.
# Para una covarianza hace falta la desviacion estandar: 95 % ~ 1,96 sigma.
_SIGMA_POR_ERR95 = 1.0 / 1.96


class GnssHeadingGpsd(Node):
    """Lee $GNHPR del socket de gpsd y republica rumbo + velocidad."""

    def __init__(self) -> None:
        """Declara parametros, abre el socket de gpsd y crea los publicadores."""
        super().__init__('gnss_heading_gpsd')

        self.declare_parameter('gpsd_host', 'localhost')
        self.declare_parameter('gpsd_port', 2947)
        self.declare_parameter('frame_id', 'gps_link')
        # true: sellar con la hora UTC que trae la propia sentencia.
        # false: con el reloj del host al leerla. Mismo criterio que /fix.
        self.declare_parameter('use_gps_time', True)
        # Cada cuanto se vacia el buffer del socket. Solo acota la latencia de
        # lectura; no cambia cuantos mensajes salen (uno por sentencia).
        self.declare_parameter('poll_rate_hz', 50.0)
        self.declare_parameter('publish_velocity', True)

        self._host: str = self.get_parameter('gpsd_host').value
        self._port: int = self.get_parameter('gpsd_port').value
        self._frame_id: str = self.get_parameter('frame_id').value
        self._use_gps_time: bool = self.get_parameter('use_gps_time').value
        poll_hz: float = self.get_parameter('poll_rate_hz').value

        self._sock: Optional[socket.socket] = None
        self._buf = b''
        self._descartadas = 0
        self._publicadas = 0
        self._aviso_sin_solucion = False

        self._heading_pub = self.create_publisher(Imu, '/gnss/heading', 10)
        self._velocity_pub = None
        if self.get_parameter('publish_velocity').value:
            self._velocity_pub = self.create_publisher(
                TwistWithCovarianceStamped, '/gnss/velocity', 10)
            self.create_subscription(GPSFix, '/extended_fix', self._on_fix, 10)

        self._conectar()
        self.create_timer(1.0 / poll_hz, self._on_timer)
        self.create_timer(10.0, self._on_estado)

    # ---------------------------------------------------------------- gpsd ---
    def _conectar(self) -> bool:
        """Abre el socket de gpsd y pide el flujo NMEA crudo. No lanza."""
        self._cerrar()
        try:
            self._sock = socket.create_connection((self._host, self._port), timeout=3.0)
            # raw:2 = pasar los bytes del dispositivo tal como llegan, que es lo
            # que hace `gpspipe -r`. Sin esto gpsd solo manda su JSON, que no
            # incluye HPR.
            self._sock.sendall(b'?WATCH={"enable":true,"raw":2};\n')
            self._sock.setblocking(False)
            self._buf = b''
            self.get_logger().info(
                f'Conectado a gpsd en {self._host}:{self._port}, leyendo NMEA crudo')
            return True
        except OSError as exc:
            self.get_logger().warn(
                f'No pude conectar a gpsd en {self._host}:{self._port}: {exc!r}. '
                f'Reintento en el proximo ciclo.')
            self._cerrar()
            return False

    def _cerrar(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _on_timer(self) -> None:
        if self._sock is None:
            self._conectar()
            return
        try:
            datos = self._sock.recv(8192)
        except BlockingIOError:
            return
        except OSError as exc:
            self.get_logger().warn(f'Se corto la lectura de gpsd: {exc!r}. Reconecto.')
            self._cerrar()
            return
        if not datos:                      # gpsd cerro la conexion
            self.get_logger().warn('gpsd cerro la conexion. Reconecto.')
            self._cerrar()
            return

        self._buf += datos
        # Se procesa por lineas COMPLETAS: un recv() puede cortar una sentencia
        # por la mitad, y media sentencia falla el checksum y se descartaria
        # como si el dato fuera malo.
        *lineas, self._buf = self._buf.split(b'\n')
        for cruda in lineas:
            self._on_linea(cruda.decode('ascii', errors='replace'))

    def _on_linea(self, linea: str) -> None:
        if ths_invalida(linea):
            if not self._aviso_sin_solucion:
                self.get_logger().warn(
                    '$--THS reporta modo V (rumbo NO valido): el receptor no '
                    'tiene solucion de doble antena. Reviso las dos antenas y '
                    'su linea base.')
                self._aviso_sin_solucion = True
            return

        hpr = parse_hpr(linea)
        if hpr is None:
            return
        if not hpr.usable:
            self._descartadas += 1
            return

        # GNHPR da rumbo de brujula (0=N, horario). REP-103 quiere yaw ENU
        # (0=Este, antihorario). Misma conversion que en um982_heading_node.
        yaw_enu_deg = 90.0 - hpr.heading_deg
        qx, qy, qz, qw = euler_deg_to_quaternion(0.0, hpr.pitch_deg, yaw_enu_deg)

        msg = Imu()
        if self._use_gps_time:
            t = epoch_desde_tod(hpr.tod_s, self.get_clock().now().nanoseconds * 1e-9)
            msg.header.stamp.sec = int(t)
            msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # El alabeo NO se paso a la conversion (se mando 0.0) y ademas se marca
        # como no observable, para que nadie lo lea como "alabeo medido = 0".
        msg.orientation_covariance[0] = _ALABEO_NO_OBSERVABLE
        msg.angular_velocity_covariance[0] = _NO_ESTIMADO
        msg.linear_acceleration_covariance[0] = _NO_ESTIMADO

        self._heading_pub.publish(msg)
        self._publicadas += 1

    # ------------------------------------------------------------ velocidad ---
    def _on_fix(self, fix: GPSFix) -> None:
        """Convierte speed/track/climb de /extended_fix en un Twist ENU."""
        if self._velocity_pub is None:
            return
        if math.isnan(fix.speed) or math.isnan(fix.track):
            return

        rumbo = math.radians(fix.track)
        msg = TwistWithCovarianceStamped()
        # Se hereda el header del fix: es la MISMA medicion, no una nueva.
        msg.header.stamp = fix.header.stamp
        msg.header.frame_id = self._frame_id
        msg.twist.twist.linear.x = fix.speed * math.sin(rumbo)   # este
        msg.twist.twist.linear.y = fix.speed * math.cos(rumbo)   # norte
        msg.twist.twist.linear.z = 0.0 if math.isnan(fix.climb) else fix.climb

        sigma_h = (fix.err_speed * _SIGMA_POR_ERR95
                   if fix.err_speed and not math.isnan(fix.err_speed) else 0.0)
        sigma_v = (fix.err_climb * _SIGMA_POR_ERR95
                   if fix.err_climb and not math.isnan(fix.err_climb) else 0.0)
        cov = [0.0] * 36
        cov[0] = sigma_h ** 2
        cov[7] = sigma_h ** 2
        cov[14] = sigma_v ** 2
        # Este mensaje no lleva velocidad angular.
        cov[21] = cov[28] = cov[35] = _ALABEO_NO_OBSERVABLE
        msg.twist.covariance = cov

        self._velocity_pub.publish(msg)

    # --------------------------------------------------------------- estado ---
    def _on_estado(self) -> None:
        if self._publicadas == 0:
            self.get_logger().warn(
                'Ninguna $--HPR valida en los ultimos 10 s. Verifica que el '
                'receptor las emita:  gpspipe -r -n 400 | grep HPR')
        elif self._descartadas:
            self.get_logger().info(
                f'{self._publicadas} rumbos publicados, {self._descartadas} '
                f'descartados por falta de solucion (QF=0).')
        self._publicadas = 0
        self._descartadas = 0


def main(args=None) -> None:
    """Punto de entrada del ejecutable."""
    rclpy.init(args=args)
    node = GnssHeadingGpsd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cerrar()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
