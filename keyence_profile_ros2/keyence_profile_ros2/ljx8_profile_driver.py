"""ROS 2 polling driver for an LJ-X8k controller via the vendor Linux SDK."""

import math
import struct
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32, Float32MultiArray

from .ljx8_protocol import Ljx8Client


def profile_diagnostic_state(
    client_connected: bool,
    last_profile_monotonic: float | None,
    now_monotonic: float,
    max_input_age: float,
    last_error: str,
) -> tuple[int, str, float]:
    """Return a fail-closed profile-receipt state independent of ROS clocks."""
    if last_profile_monotonic is None:
        return DiagnosticStatus.WARN, last_error, float('inf')
    age = max(0.0, now_monotonic - last_profile_monotonic)
    if not client_connected:
        return DiagnosticStatus.WARN, last_error, age
    if age > max(0.0, max_input_age):
        return DiagnosticStatus.WARN, f'stale profile: {last_error}', age
    return DiagnosticStatus.OK, 'streaming', age


class Ljx8ProfileDriver(Node):
    def __init__(self) -> None:
        super().__init__('ljx8_profile_driver')
        for name, default in (
            ('library_path', ''), ('host', ''), ('port', 24691), ('device_id', 0),
            ('rate', 40.0), ('max_points', 3200), ('with_luminance', False),
            ('start_measure', False), ('frame_id', 'keyence_frame'),
            ('profiles_topic', '/profiles'), ('raw_topic', '/profiles_float'),
            ('pitch_topic', '/profiles_pitch_m'),
            ('max_input_age', 0.5), ('reconnect_delay', 1.0),
        ):
            self.declare_parameter(name, default)
        self._client = None
        self._last_error = 'not connected'
        self._last_profile_monotonic = None
        self._next_connect_monotonic = 0.0
        self._cloud_pub = self.create_publisher(PointCloud2, str(self.get_parameter('profiles_topic').value), 10)
        self._raw_pub = self.create_publisher(Float32MultiArray, str(self.get_parameter('raw_topic').value), 10)
        self._pitch_pub = self.create_publisher(Float32, str(self.get_parameter('pitch_topic').value), 1)
        self._diagnostics = self.create_publisher(DiagnosticArray, '~/diagnostics', 10)
        rate = max(1.0, float(self.get_parameter('rate').value))
        self.create_timer(1.0 / rate, self._poll)
        self.create_timer(0.5, self._publish_diagnostics)

    def _connect(self) -> bool:
        if time.monotonic() < self._next_connect_monotonic:
            return False
        path = str(self.get_parameter('library_path').value)
        host = str(self.get_parameter('host').value)
        if not path or not host:
            self._last_error = 'library_path and host parameters are required'
            self._schedule_reconnect()
            return False
        try:
            self._client = Ljx8Client(path, int(self.get_parameter('device_id').value))
            self._client.connect(host, int(self.get_parameter('port').value), bool(self.get_parameter('start_measure').value))
            self._last_error = 'streaming'
            return True
        except (OSError, RuntimeError, ValueError) as error:
            self._client = None
            self._last_error = str(error)
            self._schedule_reconnect()
            self.get_logger().warning(f'Keyence connect failed: {error}')
            return False

    def _schedule_reconnect(self) -> None:
        self._next_connect_monotonic = time.monotonic() + max(
            0.1, float(self.get_parameter('reconnect_delay').value))

    def _poll(self) -> None:
        if self._client is None and not self._connect():
            return
        try:
            profile = self._client.read_profile(
                int(self.get_parameter('max_points').value),
                bool(self.get_parameter('with_luminance').value))
        except (OSError, RuntimeError, ValueError, struct.error) as error:
            self._last_error = str(error)
            self.get_logger().warning(f'Keyence profile read failed: {error}')
            try:
                self._client.close(bool(self.get_parameter('start_measure').value))
            except (OSError, RuntimeError) as close_error:
                self.get_logger().warning(f'Keyence cleanup failed: {close_error}')
            self._client = None
            self._schedule_reconnect()
            return
        self._raw_pub.publish(Float32MultiArray(data=list(profile.z_m)))
        self._pitch_pub.publish(Float32(data=profile.x_pitch_m))
        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = str(self.get_parameter('frame_id').value)
        cloud.height, cloud.width = 1, len(profile.z_m)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian, cloud.point_step = False, 12
        cloud.row_step = cloud.width * cloud.point_step
        cloud.is_dense = not any(math.isnan(z) for z in profile.z_m)
        cloud.data = struct.pack('<%df' % (cloud.width * 3),
                                 *(coordinate for point in profile.points() for coordinate in point))
        self._cloud_pub.publish(cloud)
        self._last_error = 'streaming'
        self._last_profile_monotonic = time.monotonic()

    def _publish_diagnostics(self) -> None:
        status = DiagnosticStatus(name='LJ-X8k profile driver', hardware_id='keyence')
        status.level, status.message, age = profile_diagnostic_state(
            self._client is not None,
            self._last_profile_monotonic,
            time.monotonic(),
            float(self.get_parameter('max_input_age').value),
            self._last_error,
        )
        status.values = [
            KeyValue(key='host', value=str(self.get_parameter('host').value)),
            KeyValue(key='vendor_library', value=str(self.get_parameter('library_path').value)),
            KeyValue(key='profile_age_sec', value=str(age)),
            KeyValue(key='max_input_age_sec', value=str(self.get_parameter('max_input_age').value)),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diagnostics.publish(array)

    def destroy_node(self):
        if self._client is not None:
            self._client.close(bool(self.get_parameter('start_measure').value))
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ljx8ProfileDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
