import ctypes
import math

import pytest
from diagnostic_msgs.msg import DiagnosticStatus

from keyence_profile_ros2.ljx8_protocol import (
    INVALID_PROFILE_VALUE,
    EthernetConfig,
    Ljx8Client,
    ProfileHeader,
    ProfileInfo,
    profile_from_vendor_data,
)
from keyence_profile_ros2.ljx8_profile_driver import profile_diagnostic_state


class FakeSdkFunction:
    def __init__(self, callback):
        self.callback = callback
        self.restype = None
        self.argtypes = None

    def __call__(self, *args):
        return self.callback(*args)


class FakeSdk:
    def __init__(self, *, start_result=0):
        self.calls = []
        self.start_result = start_result
        self.LJX8IF_EthernetOpen = FakeSdkFunction(self._open)
        self.LJX8IF_CommunicationClose = FakeSdkFunction(self._close)
        self.LJX8IF_StartMeasure = FakeSdkFunction(self._start)
        self.LJX8IF_StopMeasure = FakeSdkFunction(self._stop)
        self.LJX8IF_GetProfile = FakeSdkFunction(self._get_profile)

    def _open(self, device_id, config):
        endpoint = ctypes.cast(config, ctypes.POINTER(EthernetConfig)).contents
        self.calls.append(('open', device_id, tuple(endpoint.ip_address), endpoint.port))
        return 0

    def _close(self, device_id):
        self.calls.append(('close', device_id))
        return 0

    def _start(self, device_id):
        self.calls.append(('start', device_id))
        return self.start_result

    def _stop(self, device_id):
        self.calls.append(('stop', device_id))
        return 0

    def _get_profile(self, device_id, _request, _response, info, buffer, _bytes):
        self.calls.append(('get_profile', device_id))
        profile_info = ctypes.cast(info, ctypes.POINTER(ProfileInfo)).contents
        profile_info.data_count = 3
        profile_info.x_start = -100
        profile_info.x_pitch = 10
        values = ctypes.cast(buffer, ctypes.POINTER(ctypes.c_int))
        offset = ctypes.sizeof(ProfileHeader) // ctypes.sizeof(ctypes.c_int)
        for index, value in enumerate((100, INVALID_PROFILE_VALUE, -50)):
            values[offset + index] = value
        return 0


def test_converts_ljx8_units_and_invalid_return_to_common_contract():
    info = ProfileInfo()
    info.data_count = 3
    info.x_start = -1_000_000
    info.x_pitch = 500_000
    profile = profile_from_vendor_data(info, [200_000, INVALID_PROFILE_VALUE, -100_000])
    assert profile.x_start_m == -0.01
    assert profile.x_pitch_m == 0.005
    assert profile.points()[0] == (-0.01, 0.0, 0.002)
    assert math.isnan(profile.z_m[1])
    assert profile.z_m[2] == -0.001


def test_rejects_short_or_empty_vendor_profile():
    info = ProfileInfo()
    info.data_count = 2
    with pytest.raises(ValueError, match='invalid'):
        profile_from_vendor_data(info, [1])


def test_fake_sdk_exercises_open_start_read_and_close_without_controller():
    sdk = FakeSdk()
    client = Ljx8Client('/unused/libljxacom.so', device_id=2, library=sdk)
    client.connect('192.168.0.10', 24691, start_measure=True)
    profile = client.read_profile(16)
    client.close(stop_measure=True)

    assert sdk.calls == [
        ('open', 2, (192, 168, 0, 10), 24691), ('start', 2),
        ('get_profile', 2), ('stop', 2), ('close', 2),
    ]
    assert profile.x_start_m == -1.0e-6
    assert profile.x_pitch_m == 1.0e-7
    assert profile.z_m[0] == 1.0e-6
    assert math.isnan(profile.z_m[1])
    assert profile.z_m[2] == -5.0e-7


@pytest.mark.parametrize('host, port', [
    ('192.168.0.256', 24691), ('192.168.0', 24691), ('192.168.0.1', 0),
    ('192.168.0.1', 65536), ('192.168.x.1', 24691),
])
def test_rejects_invalid_keyence_endpoint_before_vendor_call(host, port):
    sdk = FakeSdk()
    client = Ljx8Client('/unused/libljxacom.so', library=sdk)
    with pytest.raises(ValueError):
        client.connect(host, port)
    assert sdk.calls == []


def test_start_measure_failure_closes_connection():
    sdk = FakeSdk(start_result=0x12)
    client = Ljx8Client('/unused/libljxacom.so', library=sdk)
    with pytest.raises(RuntimeError, match='StartMeasure'):
        client.connect('10.0.0.2', 24691, start_measure=True)
    assert sdk.calls == [
        ('open', 0, (10, 0, 0, 2), 24691), ('start', 0), ('close', 0),
    ]


def test_profile_diagnostic_state_requires_fresh_data_not_only_a_connection():
    level, message, age = profile_diagnostic_state(True, None, 10.0, 0.5, 'streaming')
    assert (level, message, age) == (DiagnosticStatus.WARN, 'streaming', float('inf'))

    level, message, age = profile_diagnostic_state(True, 9.0, 10.0, 0.5, 'streaming')
    assert (level, message, age) == (DiagnosticStatus.WARN, 'stale profile: streaming', 1.0)

    level, message, age = profile_diagnostic_state(True, 9.8, 10.0, 0.5, 'streaming')
    assert level == DiagnosticStatus.OK
    assert message == 'streaming'
    assert age == pytest.approx(0.2)
