"""Demosaico Bayer -> RGB para visualizacion.

Por que un nodo propio y no image_proc:
  - image_proc es un paquete aparte que puede no estar instalado, y un Node
    apuntando a un paquete inexistente ABORTA el launch entero.
  - Aca hace falta controlar el ritmo: las camaras entregan 3x3,15 MB a 12 fps
    (113 MB/s de Bayer). Demosaicar todo produciria 3x9,44 MB a 12 fps = 340
    MB/s de RGB solo para mirar la pantalla, justo el trafico que se quiso
    evitar. Para VERIFICAR que la camara funciona alcanza con unos pocos fps,
    asi que este nodo limita la tasa (max_rate_hz) y descarta el resto sin
    convertirlo.

Entrada : `image`        (sensor_msgs/Image, encoding bayer_*)
Salida  : `image_color`  (sensor_msgs/Image, rgb8)

Si la entrada NO es Bayer (p.ej. ya viene en rgb8/mono8), se reenvia tal cual:
asi la vista funciona igual si se vuelve a pixelformat:=rgb8.

Parametros:
  max_rate_hz  (5.0)  frames/segundo a convertir. <=0 = todos.
  qos_depth    (1)    cola corta: para ver, interesa el frame mas reciente.
"""
import time

import cv2

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image

# OpenCV nombra el patron Bayer desplazado respecto a ROS, asi que el mapeo
# parece cruzado pero es el correcto (es el mismo que usa image_proc):
#   ROS bayer_rggb8 -> cv2.COLOR_BayerBG2RGB
_BAYER_TO_CV = {
    'bayer_rggb8': cv2.COLOR_BayerBG2RGB,
    'bayer_bggr8': cv2.COLOR_BayerRG2RGB,
    'bayer_gbrg8': cv2.COLOR_BayerGR2RGB,
    'bayer_grbg8': cv2.COLOR_BayerGB2RGB,
}


class DebayerNode(Node):
    """Convierte imagenes Bayer a rgb8, limitando la tasa de conversion."""

    def __init__(self) -> None:
        super().__init__('debayer')

        self.declare_parameter('max_rate_hz', 5.0)
        self.declare_parameter('qos_depth', 1)

        self._max_rate = float(self.get_parameter('max_rate_hz').value)
        depth = int(self.get_parameter('qos_depth').value)
        self._min_period = 1.0 / self._max_rate if self._max_rate > 0 else 0.0
        # None (no 0.0) para que el PRIMER frame pase siempre.
        self._last_stamp = None
        self._warned_encoding = None
        self._seen = 0
        self._published = 0

        # QoS asimetrico a proposito:
        #   suscripcion BEST_EFFORT -- un suscriptor BEST_EFFORT matchea con
        #     publicadores RELIABLE *y* BEST_EFFORT; uno RELIABLE solo matchea
        #     con RELIABLE. Suscribirse RELIABLE ataba este nodo a que la
        #     camara (y el `ros2 bag play` de turno) ofrecieran exactamente esa
        #     politica: si alguna vez no lo hacen, el nodo arranca, no recibe
        #     nada, y no hay error que lo explique.
        #   publicacion RELIABLE -- por el mismo motivo del otro lado: le sirve
        #     a RViz sea cual sea su politica.
        # Aca no se pierde nada por elegir BEST_EFFORT: este nodo ya descarta
        # frames a proposito (max_rate_hz), es para mirar, no para grabar.
        sub_qos = QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub_qos = QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE)

        self._pub = self.create_publisher(Image, 'image_color', pub_qos)
        self._sub = self.create_subscription(Image, 'image', self._on_image, sub_qos)

        self.get_logger().info(
            'debayer listo en ns=%s: image -> image_color (max %.1f Hz)'
            % (self.get_namespace(), self._max_rate))

    def _on_image(self, msg: Image) -> None:
        # Diagnostico: confirmar que los frames LLEGAN (si esto no aparece, el
        # problema esta en el topico/QoS, no en la conversion).
        self._seen += 1
        if self._seen == 1:
            self.get_logger().info(
                'primer frame recibido: %dx%d encoding=%s step=%d'
                % (msg.width, msg.height, msg.encoding, msg.step))

        # Limitador de tasa: descartar temprano, sin convertir.
        # Se usa time.monotonic() y NO el reloj ROS a proposito: esto limita
        # cuanto trabaja la CPU, que es tiempo real. Con use_sim_time y sin
        # /clock el reloj ROS se queda clavado en 0 y el nodo descartaria todos
        # los frames para siempre, quedando mudo sin ningun error.
        if self._min_period > 0.0:
            now = time.monotonic()
            if self._last_stamp is not None and \
                    now - self._last_stamp < self._min_period:
                return
            self._last_stamp = now

        code = _BAYER_TO_CV.get(msg.encoding)
        if code is None:
            # No es Bayer: reenviar sin tocar, para que la vista siga andando
            # si alguien vuelve a pixelformat:=rgb8.
            if self._warned_encoding != msg.encoding:
                self.get_logger().info(
                    "encoding '%s' no es Bayer: se reenvia sin convertir"
                    % msg.encoding)
                self._warned_encoding = msg.encoding
            self._pub.publish(msg)
            return

        try:
            # Se usa msg.step (bytes por fila) y no msg.width, por si el
            # driver alguna vez alinea las filas con padding.
            step = msg.step if msg.step else msg.width
            mosaic = np.frombuffer(
                msg.data, dtype=np.uint8).reshape(msg.height, step)[:, :msg.width]
            rgb = cv2.cvtColor(mosaic, code)
        except Exception as exc:  # noqa: BLE001 - no tirar el nodo por un frame
            self.get_logger().warn(
                'no se pudo convertir el frame (%dx%d, step=%d, %s): %r'
                % (msg.width, msg.height, msg.step, msg.encoding, exc),
                throttle_duration_sec=5.0)
            return

        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = 'rgb8'
        out.is_bigendian = 0
        out.step = msg.width * 3
        out.data = rgb.tobytes()
        self._pub.publish(out)

        self._published += 1
        if self._published == 1:
            self.get_logger().info(
                'primer frame convertido y publicado en image_color '
                '(%dx%d rgb8). Si RViz sigue sin mostrarlo, revisa el topico '
                'del display.' % (out.width, out.height))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DebayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
