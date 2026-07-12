"""
ROS 2 node publishing GNSS velocity and dual-antenna heading from a Unicore UM982.

Position (NavSatFix) is deliberately NOT published here: gpsd_client (see
gps_bringup) already owns /fix with PPS-disciplined timestamps for the PTP
sync pipeline, and this node must not contend for that serial port or that
responsibility. This node only adds velocity and heading, which gpsd cannot
parse from the UM982's proprietary PVTSLNA/BESTNAVA/HPR logs.
"""
import math
import os
import time
from typing import Optional, Tuple

from geometry_msgs.msg import TwistWithCovarianceStamped

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

from um982.UM982 import UM982Serial

# um982-driver's UM982Serial.__init__ reads exactly 10 lines and assumes a
# #PVTSLNA sentence showed up among them; with three interleaved sentence
# types on one port that isn't guaranteed, and it raises TypeError instead of
# retrying. Since we depend on this library unmodified (GPL-3.0, not
# vendored), we retry construction here rather than patching it.
_CONNECT_RETRY_INITIAL_S = 1.0
_CONNECT_RETRY_MAX_S = 5.0

# Twist has no standard "unavailable" covariance marker (unlike sensor_msgs/Imu,
# which reserves -1 in covariance[0] for that). The UM982 doesn't report
# angular velocity, so mark it with a large variance to signal "unreliable /
# disregard" rather than an artificially small, misleadingly confident one.
_UNMEASURED_ANGULAR_VARIANCE = 1e6


def euler_deg_to_quaternion(
    roll_deg: float, pitch_deg: float, yaw_deg: float,
) -> Tuple[float, float, float, float]:
    """Convert roll/pitch/yaw (degrees, REP-103 body frame) to a quaternion (x, y, z, w)."""
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


class Um982HeadingNode(Node):
    """Polls a UM982Serial instance and republishes velocity + heading on ROS 2 topics."""

    def __init__(self) -> None:
        """Declare parameters, connect to the UM982, and set up publishers/timer."""
        super().__init__('um982_heading_node')

        self.declare_parameter('port', '/dev/um982_heading')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('publish_rate_hz', 10.0)

        self._port: str = self.get_parameter('port').value
        self._baud: int = self.get_parameter('baud').value
        self._frame_id: str = self.get_parameter('frame_id').value
        publish_rate_hz: float = self.get_parameter('publish_rate_hz').value

        if self._port == '/dev/um982_heading' and not os.path.exists(self._port):
            self.get_logger().warn(
                "Parameter 'port' is the default '/dev/um982_heading' but that symlink "
                "doesn't exist yet. Add the matching udev rule (see launch/README) with "
                "the real USB serial number for the second, dedicated UM982 link -- "
                "distinct from gpsd's /dev/gps_pps."
            )

        self._driver: Optional[UM982Serial] = None
        self._connect()

        timer_period_s = 1.0 / publish_rate_hz
        self._velocity_pub = self.create_publisher(
            TwistWithCovarianceStamped, '/gnss/velocity', 10)
        self._heading_pub = self.create_publisher(Imu, '/gnss/heading', 10)
        self._timer = self.create_timer(timer_period_s, self._on_timer)

    def _connect(self) -> None:
        backoff_s = _CONNECT_RETRY_INITIAL_S
        while rclpy.ok():
            try:
                self._driver = UM982Serial(self._port, self._baud)
                self._driver.start()
                self.get_logger().info(
                    f'Connected to UM982 on {self._port} @ {self._baud}')
                return
            except Exception as exc:  # noqa: BLE001 - third-party driver, unknown exceptions
                self.get_logger().warn(
                    f'Failed to connect/init UM982Serial on {self._port}: {exc!r}. '
                    f'Retrying in {backoff_s:.1f}s.',
                )
                time.sleep(backoff_s)
                backoff_s = min(backoff_s * 2.0, _CONNECT_RETRY_MAX_S)

    def _on_timer(self) -> None:
        assert self._driver is not None
        # NOTE (known limitation, T1 scope): timestamp is software time at the
        # moment this node reads the already-parsed sample, not hardware/PTP
        # time. Unlike gpsd's /fix, the UM982's own log timestamps are not used
        # here. Integrating with the PTP/PPS work is explicitly future work,
        # not resolved by this node.
        now = self.get_clock().now().to_msg()

        vel = self._driver.vel
        if vel is not None:
            self._publish_velocity(now, vel)

        orientation = self._driver.orientation
        if orientation is not None:
            self._publish_heading(now, orientation)

    def _publish_velocity(
        self, stamp, vel: Tuple[float, float, float, float, float, float],
    ) -> None:
        """Publish a TwistWithCovarianceStamped built from the driver's ENU velocity tuple."""
        vel_east, vel_north, vel_up, vel_hor_std, _vel_hor_std_dup, vel_ver_std = vel

        msg = TwistWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.twist.twist.linear.x = vel_east
        msg.twist.twist.linear.y = vel_north
        msg.twist.twist.linear.z = vel_up

        cov = [0.0] * 36
        cov[0] = vel_hor_std ** 2   # vx (east) variance
        cov[7] = vel_hor_std ** 2   # vy (north) variance -- driver reuses horizontal std for both
        cov[14] = vel_ver_std ** 2  # vz (up) variance
        cov[21] = _UNMEASURED_ANGULAR_VARIANCE
        cov[28] = _UNMEASURED_ANGULAR_VARIANCE
        cov[35] = _UNMEASURED_ANGULAR_VARIANCE
        msg.twist.covariance = cov

        self._velocity_pub.publish(msg)

    def _publish_heading(self, stamp, orientation: Tuple[float, float, float]) -> None:
        """Publish an orientation-only Imu built from the driver's (heading, pitch, roll) tuple."""
        heading_deg, pitch_deg, roll_deg = orientation

        # GNHPR heading is true-north-referenced, clockwise-positive (compass
        # convention: 0=N, 90=E). Convert to REP-103 ENU yaw (0=East,
        # counter-clockwise-positive) for the quaternion below.
        yaw_enu_deg = 90.0 - heading_deg

        # Pitch/roll sign convention from GNHPR is passed through as-is
        # (degrees -> radians only). um982-driver does not document their
        # sign relative to REP-103's body frame, and it was not verified
        # against the Unicore manual here -- confirm during T3 field
        # verification by physically rotating/tilting the antenna baseline
        # and checking the sign matches expectation before trusting pitch/roll.
        qx, qy, qz, qw = euler_deg_to_quaternion(roll_deg, pitch_deg, yaw_enu_deg)

        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # No per-sample heading/pitch/roll accuracy is exposed by um982-driver,
        # so we don't invent a covariance number: -1 marks it unavailable per
        # the sensor_msgs/Imu convention (see msg comment), same as below.
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self._heading_pub.publish(msg)

    def destroy_node(self) -> bool:
        """Stop the UM982Serial background thread before tearing down the ROS node."""
        if self._driver is not None:
            self._driver.stop()
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    """Run the Um982HeadingNode until interrupted."""
    rclpy.init(args=args)
    node = Um982HeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
