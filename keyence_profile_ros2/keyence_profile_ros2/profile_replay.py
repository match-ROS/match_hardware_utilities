"""Publish recorded Keyence profiles using the ROS 1 print topic contract."""

import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32, Float32MultiArray

from .profile import parse_profiles


def _time_from_seconds(seconds):
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1_000_000_000))
    if nanosec == 1_000_000_000:
        sec += 1
        nanosec = 0
    return sec, nanosec


class KeyenceProfileReplay(Node):
    def __init__(self):
        super().__init__('keyence_profile_replay')
        fixture_file = self.declare_parameter('fixture_file', '').value
        self._profiles = parse_profiles(fixture_file)
        self._loop = self.declare_parameter('loop', False).value
        rate = float(self.declare_parameter('publish_rate', 20.0).value)
        if rate <= 0.0:
            raise ValueError('publish_rate must be positive')
        self._cloud_pub = self.create_publisher(
            PointCloud2, self.declare_parameter('profiles_topic', '/profiles').value, 10)
        self._raw_pub = self.create_publisher(
            Float32MultiArray, self.declare_parameter('raw_topic', '/profiles_float').value, 10)
        self._pitch_pub = self.create_publisher(
            Float32, self.declare_parameter('pitch_topic', '/profiles_pitch_m').value, 1)
        self._index = 0
        self._timer = self.create_timer(1.0 / rate, self._publish_next)

    def _publish_next(self):
        profile = self._profiles[self._index]
        self._pitch_pub.publish(Float32(data=profile.x_pitch_m))
        self._raw_pub.publish(Float32MultiArray(data=list(profile.z_m)))

        cloud = PointCloud2()
        sec, nanosec = _time_from_seconds(profile.stamp)
        cloud.header.stamp.sec = sec
        cloud.header.stamp.nanosec = nanosec
        cloud.header.frame_id = profile.frame_id
        cloud.height = 1
        cloud.width = len(profile.z_m)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = not any(math.isnan(value) for value in profile.z_m)
        cloud.data = struct.pack('<%df' % (cloud.width * 3),
                                 *(coordinate for point in profile.points() for coordinate in point))
        self._cloud_pub.publish(cloud)

        self._index += 1
        if self._index == len(self._profiles):
            if self._loop:
                self._index = 0
            else:
                self._timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = KeyenceProfileReplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
