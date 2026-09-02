"""
ROS 2 node that tees the UM982's raw binary stream to disk for offline PPK.

Captures OBSVMB (observations) + the ephemeris/ionosphere logs enabled
receiver-side via `configure_um982 --enable-ppk-raw` -- see
PPK_IMPLEMENTATION.md for the post-process (convbin/rnx2rtkp) steps.

Shares the dedicated /dev/um982_heading port with um982_heading_node -- the
two are MUTUALLY EXCLUSIVE, same rule as um982_heading_node vs
gnss_heading_gpsd_node on /gnss/heading. Never run both against the same
port at once.

This node does not parse anything: OBSVMB/*EPHB/*IONB are Unicore binary
frames, not NMEA, and the third-party um982-driver library doesn't decode
them either (RTKLIB's convbin does, offline). So the node just tees the
bytes it reads straight to a file -- no dependency on um982-driver, unlike
um982_heading_node.
"""
import os
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node

import serial

from um982_driver.raw_log_session import raw_file_path, resolve_session_name

_CONNECT_RETRY_INITIAL_S = 1.0
_CONNECT_RETRY_MAX_S = 5.0


class Um982RawLogNode(Node):
    """Copia a disco, sin parsear, los bytes crudos leidos del puerto dedicado del UM982."""

    def __init__(self) -> None:
        """Declare parameters and, only if raw_log_enabled, open the port and start capture."""
        super().__init__('um982_raw_log_node')

        self.declare_parameter('port', '/dev/um982_heading')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('raw_log_enabled', False)
        self.declare_parameter('raw_log_dir', '/data/eod_av/gnss_raw')
        self.declare_parameter('session_name', '')
        self.declare_parameter('read_timeout_s', 0.5)
        self.declare_parameter('read_chunk_bytes', 4096)
        self.declare_parameter('status_log_period_s', 10.0)

        self._port: str = self.get_parameter('port').value
        self._baud: int = self.get_parameter('baud').value
        self._read_timeout_s: float = self.get_parameter('read_timeout_s').value
        self._read_chunk_bytes: int = self.get_parameter('read_chunk_bytes').value
        raw_log_enabled: bool = self.get_parameter('raw_log_enabled').value
        status_log_period_s: float = self.get_parameter('status_log_period_s').value

        self._ser: Optional[serial.Serial] = None
        self._raw_file = None
        self._bytes_written = 0
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        # Gate defensivo: si esto arranca con raw_log_enabled=false (default),
        # no toca el puerto ni el disco para nada. El gate principal es el
        # IfCondition del launch file -- este es el respaldo para quien corra
        # `ros2 run` directo. Sin esto, un raw-logger prendido por error
        # pelearia por el mismo puerto que um982_heading_node, la misma clase
        # de falla que ya paso una vez con el puerto de gpsd.
        if not raw_log_enabled:
            self.get_logger().info(
                'raw_log_enabled=false -- nodo inactivo, no abre el puerto ni crea '
                'archivos. Activalo con raw_log_enabled:=true despues de correr '
                'configure_um982 --enable-ppk-raw contra el receptor.')
            return

        raw_log_dir: str = self.get_parameter('raw_log_dir').value
        session_name = resolve_session_name(self.get_parameter('session_name').value)
        os.makedirs(raw_log_dir, exist_ok=True)
        file_path = raw_file_path(raw_log_dir, session_name)
        self._raw_file = open(file_path, 'wb', buffering=0)
        self.get_logger().info(f'Grabando crudo en {file_path}')

        if self._port == '/dev/um982_heading' and not os.path.exists(self._port):
            self.get_logger().warn(
                "Parameter 'port' is the default '/dev/um982_heading' but that symlink "
                "doesn't exist yet. Same udev rule as um982_heading_node -- see README.")

        self._connect()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        if status_log_period_s > 0.0:
            self.create_timer(status_log_period_s, self._on_status_timer)

    def _connect(self) -> None:
        backoff_s = _CONNECT_RETRY_INITIAL_S
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                self._ser = serial.Serial(self._port, self._baud, timeout=self._read_timeout_s)
                self.get_logger().info(
                    f'Connected to UM982 on {self._port} @ {self._baud} for raw capture')
                return
            except serial.SerialException as exc:
                self.get_logger().warn(
                    f'Failed to open {self._port}: {exc!r}. Retrying in {backoff_s:.1f}s.')
                time.sleep(backoff_s)
                backoff_s = min(backoff_s * 2.0, _CONNECT_RETRY_MAX_S)

    def _read_loop(self) -> None:
        """Un solo read() alimenta un solo consumidor (el archivo) -- no hay nada que parsear."""
        while not self._stop_event.is_set():
            if self._ser is None:
                return
            try:
                chunk = self._ser.read(self._read_chunk_bytes)
            except serial.SerialException as exc:
                self.get_logger().warn(f'Serial read failed: {exc!r}. Reconnecting.')
                self._close_serial()
                self._connect()
                continue
            if chunk:
                self._raw_file.write(chunk)
                self._bytes_written += len(chunk)

    def _on_status_timer(self) -> None:
        self.get_logger().info(f'{self._bytes_written} bytes escritos hasta ahora.')

    def _close_serial(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except serial.SerialException:
                pass
            self._ser = None

    def destroy_node(self) -> bool:
        """Frena el hilo de lectura y cierra el puerto/archivo antes de destruir el nodo."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._close_serial()
        if self._raw_file is not None:
            self._raw_file.close()
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    """Run the Um982RawLogNode until interrupted."""
    rclpy.init(args=args)
    node = Um982RawLogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
