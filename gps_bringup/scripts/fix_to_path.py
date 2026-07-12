#!/usr/bin/env python3
"""
fix_to_path.py
Convierte sensor_msgs/NavSatFix (/fix) a:
  - nav_msgs/Path (/gps_path)  -> trayectoria en metros (frame 'map', ENU local)
  - TF map -> gps_link         -> posicion actual para RViz / rviz_satellite
"""
import math
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

        self.frame = self.get_parameter('frame_id').value
        self.child = self.get_parameter('child_frame').value
        fix_topic = self.get_parameter('fix_topic').value

        self.origin = None            # (lat0, lon0) del primer fix
        self.path = Path()
        self.path.header.frame_id = self.frame

        self.pub = self.create_publisher(Path, 'gps_path', 10)
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(NavSatFix, fix_topic, self.cb, 10)
        self.get_logger().info(f"Escuchando {fix_topic}, publicando /gps_path en '{self.frame}'")

    def cb(self, msg: NavSatFix):
        # No usamos msg.status.status: gpsd_client siempre reporta STATUS_NO_FIX
        # con gpsd >= 3.23 en fixes GPS simples (gpsd omite el campo "status" en
        # su JSON salvo fixes DGPS o mejores, ver gitlab.com/gpsd/gpsd/-/issues/154).
        # lat/lon en NaN es la unica senal fiable de "sin fix" que da gpsd_client.
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        if self.origin is None:
            self.origin = (msg.latitude, msg.longitude)
            self.get_logger().info(
                f"Origen fijado en lat={msg.latitude:.6f}, lon={msg.longitude:.6f}")

        lat0, lon0 = self.origin
        # Proyeccion ENU local (equirectangular, suficiente para pocos km)
        x = math.radians(msg.longitude - lon0) * R * math.cos(math.radians(lat0))
        y = math.radians(msg.latitude - lat0) * R
        z = msg.altitude - 120.0  # opcional: relativo, solo para que no quede muy alto

        now = self.get_clock().now().to_msg()

        # --- Path ---
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = self.frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        self.path.header.stamp = now
        self.path.poses.append(pose)
        self.pub.publish(self.path)

        # --- TF map -> gps_link (posicion actual) ---
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
