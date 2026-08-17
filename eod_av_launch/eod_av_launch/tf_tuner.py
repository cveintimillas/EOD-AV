"""Publica una TF ajustable EN CALIENTE con `ros2 param set`.

Pensado para calibrar el montaje de un sensor mirando RViz en vivo: se gira/
desplaza el frame con `ros2 param set` y el cambio se ve al instante, sin
reiniciar nada. Cuando el valor es el correcto se copia a
eod_av_launch/bringup.py (o se pasa por launch) y se puede volver a la TF
estatica.

Ejemplo (radar):

    ros2 param set /radar_tf yaw_deg -90.0      # gira el frente del radar
    ros2 param set /radar_tf x 0.35             # lo desplaza 35 cm adelante
    ros2 param get /radar_tf yaw_deg

Los parametros se releen en cada tick del timer, asi que cualquier `param set`
tiene efecto inmediato (no hace falta callback).

Nota: publica TF DINAMICA (a rate_hz). Para grabar el dataset final conviene
volver a la TF estatica (radar_tf_live:=false), que es la que va a /tf_static y
evita cualquier problema de interpolacion/extrapolacion al reproducir el bag.
"""
import math
from typing import Optional, Tuple

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


def euler_deg_to_quaternion(
    roll_deg: float, pitch_deg: float, yaw_deg: float,
) -> Tuple[float, float, float, float]:
    """Convierte roll/pitch/yaw (grados, REP-103) a cuaternion (x, y, z, w)."""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class TfTuner(Node):
    """Republica parent -> child con traslacion/rotacion tomadas de parametros."""

    def __init__(self) -> None:
        super().__init__('tf_tuner')

        self.declare_parameter('parent_frame', 'hesai_lidar')
        self.declare_parameter('child_frame', 'radar_fixed')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('roll_deg', 0.0)
        self.declare_parameter('pitch_deg', 0.0)
        self.declare_parameter('yaw_deg', 0.0)
        self.declare_parameter('rate_hz', 20.0)

        self._br = TransformBroadcaster(self)
        self._last: Optional[tuple] = None

        rate_hz = self.get_parameter('rate_hz').value
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(
            'tf_tuner listo: %s -> %s. Ajusta en caliente con '
            "`ros2 param set /%s yaw_deg <grados>`" % (
                self.get_parameter('parent_frame').value,
                self.get_parameter('child_frame').value,
                self.get_name()))

    def _on_timer(self) -> None:
        # Se releen en cada tick: un `ros2 param set` se aplica al instante.
        parent = self.get_parameter('parent_frame').value
        child = self.get_parameter('child_frame').value
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        z = float(self.get_parameter('z').value)
        roll = float(self.get_parameter('roll_deg').value)
        pitch = float(self.get_parameter('pitch_deg').value)
        yaw = float(self.get_parameter('yaw_deg').value)

        current = (parent, child, x, y, z, roll, pitch, yaw)
        if current != self._last:
            # Log solo al cambiar, para poder copiar el valor final a bringup.py.
            self.get_logger().info(
                '%s -> %s | xyz=(%.3f, %.3f, %.3f) rpy_deg=(%.2f, %.2f, %.2f)'
                % (parent, child, x, y, z, roll, pitch, yaw))
            self._last = current

        qx, qy, qz, qw = euler_deg_to_quaternion(roll, pitch, yaw)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._br.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TfTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
