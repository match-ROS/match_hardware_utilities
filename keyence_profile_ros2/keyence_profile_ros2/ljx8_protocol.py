"""Minimal LJ-X8k vendor-SDK boundary and unit-normalized profile conversion."""

import ctypes
import math
from dataclasses import dataclass


INVALID_PROFILE_VALUE = -2147483645
LENGTH_UNIT_M = 1.0e-8  # LJ-X8k SDK coordinate unit is 0.01 micrometres.


class EthernetConfig(ctypes.Structure):
    _fields_ = [('ip_address', ctypes.c_ubyte * 4), ('port', ctypes.c_ushort), ('reserve', ctypes.c_ubyte * 2)]


class ProfileInfo(ctypes.Structure):
    _fields_ = [
        ('profile_count', ctypes.c_ubyte), ('reserve1', ctypes.c_ubyte),
        ('luminance_output', ctypes.c_ubyte), ('reserve2', ctypes.c_ubyte),
        ('data_count', ctypes.c_ushort), ('reserve3', ctypes.c_ubyte * 2),
        ('x_start', ctypes.c_int), ('x_pitch', ctypes.c_int),
    ]


class GetProfileRequest(ctypes.Structure):
    _fields_ = [
        ('target_bank', ctypes.c_ubyte), ('position_mode', ctypes.c_ubyte), ('reserve', ctypes.c_ubyte * 2),
        ('profile_no', ctypes.c_uint), ('profile_count', ctypes.c_ubyte), ('erase', ctypes.c_ubyte),
        ('reserve2', ctypes.c_ubyte * 2),
    ]


class GetProfileResponse(ctypes.Structure):
    _fields_ = [
        ('current_profile_no', ctypes.c_uint), ('oldest_profile_no', ctypes.c_uint),
        ('get_top_profile_no', ctypes.c_uint), ('get_profile_count', ctypes.c_ubyte), ('reserve', ctypes.c_ubyte * 3),
    ]


class ProfileHeader(ctypes.Structure):
    _fields_ = [('reserve', ctypes.c_uint), ('trigger_count', ctypes.c_uint), ('encoder_count', ctypes.c_int), ('reserve2', ctypes.c_uint * 3)]


class ProfileFooter(ctypes.Structure):
    _fields_ = [('reserve', ctypes.c_uint)]


@dataclass(frozen=True)
class LxProfile:
    x_start_m: float
    x_pitch_m: float
    z_m: tuple[float, ...]

    def points(self):
        return tuple((self.x_start_m + index * self.x_pitch_m, 0.0, z)
                     for index, z in enumerate(self.z_m))


def profile_from_vendor_data(info: ProfileInfo, values) -> LxProfile:
    """Convert a single LJ-X8k height vector into the common SI profile contract."""
    count = int(info.data_count)
    if count <= 0 or len(values) < count:
        raise ValueError('LJ-X8k profile data count is invalid')
    z_m = tuple(math.nan if int(value) <= INVALID_PROFILE_VALUE else int(value) * LENGTH_UNIT_M
                for value in values[:count])
    return LxProfile(float(info.x_start) * LENGTH_UNIT_M, float(info.x_pitch) * LENGTH_UNIT_M, z_m)


class Ljx8Client:
    """Thin lifecycle wrapper around the vendor `libljxacom.so` SDK."""
    def __init__(self, library_path: str, device_id: int = 0, *, library=None):
        """Create a client around an explicit SDK library.

        ``library`` is a narrow test seam for an SDK-compatible fake.  Normal
        ROS use always loads the supplied vendor shared-library path; neither
        replay nor a test fake can accidentally be selected by a node parameter.
        """
        self._library = ctypes.cdll.LoadLibrary(library_path) if library is None else library
        self._device_id = device_id
        self._open = self._library.LJX8IF_EthernetOpen
        self._open.restype = ctypes.c_int
        self._open.argtypes = [ctypes.c_int, ctypes.POINTER(EthernetConfig)]
        self._close = self._library.LJX8IF_CommunicationClose
        self._close.restype = ctypes.c_int
        self._close.argtypes = [ctypes.c_int]
        self._get_profile = self._library.LJX8IF_GetProfile
        self._get_profile.restype = ctypes.c_int
        self._get_profile.argtypes = [ctypes.c_int, ctypes.POINTER(GetProfileRequest), ctypes.POINTER(GetProfileResponse), ctypes.POINTER(ProfileInfo), ctypes.c_void_p, ctypes.c_uint]
        self._start_measure = self._library.LJX8IF_StartMeasure
        self._start_measure.restype = ctypes.c_int
        self._start_measure.argtypes = [ctypes.c_int]
        self._stop_measure = self._library.LJX8IF_StopMeasure
        self._stop_measure.restype = ctypes.c_int
        self._stop_measure.argtypes = [ctypes.c_int]
        self._connected = False

    def connect(self, host: str, port: int, start_measure: bool = False) -> None:
        octets = host.split('.')
        if len(octets) != 4:
            raise ValueError('LJ-X8k host must be an IPv4 address')
        try:
            port = int(port)
        except (TypeError, ValueError) as error:
            raise ValueError('LJ-X8k port must be in range 1..65535') from error
        if not 1 <= port <= 65535:
            raise ValueError('LJ-X8k port must be in range 1..65535')
        config = EthernetConfig()
        try:
            for index, value in enumerate(octets):
                octet = int(value)
                if not 0 <= octet <= 255:
                    raise ValueError
                config.ip_address[index] = octet
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError('LJ-X8k host must be an IPv4 address') from error
        config.port = port
        result = self._open(self._device_id, ctypes.byref(config))
        if result != 0:
            raise RuntimeError(f'LJX8IF_EthernetOpen failed: 0x{result:X}')
        self._connected = True
        if start_measure:
            result = self._start_measure(self._device_id)
            if result != 0:
                self.close()
                raise RuntimeError(f'LJX8IF_StartMeasure failed: 0x{result:X}')

    def read_profile(self, max_points: int, with_luminance: bool = False) -> LxProfile:
        if not self._connected:
            raise RuntimeError('LJ-X8k client is not connected')
        header_bytes = ctypes.sizeof(ProfileHeader)
        footer_bytes = ctypes.sizeof(ProfileFooter)
        buffer_bytes = header_bytes + footer_bytes + ctypes.sizeof(ctypes.c_uint) * max_points * (1 + int(with_luminance))
        buffer = (ctypes.c_int * (buffer_bytes // ctypes.sizeof(ctypes.c_int)))()
        request, response, info = GetProfileRequest(), GetProfileResponse(), ProfileInfo()
        request.profile_count = 1
        result = self._get_profile(self._device_id, ctypes.byref(request), ctypes.byref(response), ctypes.byref(info), buffer, ctypes.c_uint(buffer_bytes))
        if result != 0:
            raise RuntimeError(f'LJX8IF_GetProfile failed: 0x{result:X}')
        offset = header_bytes // ctypes.sizeof(ctypes.c_int)
        return profile_from_vendor_data(info, buffer[offset:offset + int(info.data_count)])

    def close(self, stop_measure: bool = False) -> None:
        if not self._connected:
            return
        if stop_measure:
            self._stop_measure(self._device_id)
        self._close(self._device_id)
        self._connected = False
