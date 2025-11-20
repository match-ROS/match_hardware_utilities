#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import ctypes
import sys

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg

import LJXAwrap  # muss im PYTHONPATH liegen (wie bei den Keyence-Samples)


class KeyenceProfileNode(object):
    def __init__(self):
        rospy.init_node("keyence_ljx_profile_node")

        # --- Parameter ---
        self.device_id = rospy.get_param("~device_id", 0)
        ip_str = rospy.get_param("~ip_address", "192.168.12.88")
        self.port = rospy.get_param("~port", 24691)
        self.rate_hz = rospy.get_param("~rate", 10.0)           # Profilrate ROS-seitig
        self.xpoint_num = rospy.get_param("~xpoint_num", 3200)  # X-Punkte pro Profil
        self.with_lumi = rospy.get_param("~with_luminance", 1)  # 1 = inkl. Luminanzdaten
        self.start_measure = rospy.get_param("~start_measure", False)
        self.frame_id = rospy.get_param("~frame_id", "keyence_frame")

        # Publisher
        self.raw_pub = rospy.Publisher("/profiles", Float32MultiArray, queue_size=1)
        self.pc_pub = rospy.Publisher("/profiles_cloud", PointCloud2, queue_size=1)

        # --- Ethernet-Konfig setzen (wie in den Samples) ---
        self.eth_cfg = LJXAwrap.LJX8IF_ETHERNET_CONFIG()
        ip_parts = [int(x) for x in ip_str.split(".")]
        if len(ip_parts) != 4:
            rospy.logfatal("ip_address must have 4 octets, got: %s", ip_str)
            sys.exit(1)
        for i in range(4):
            self.eth_cfg.abyIpAddress[i] = ip_parts[i]
        self.eth_cfg.wPortNo = self.port

        # --- Verbindung öffnen ---
        res = LJXAwrap.LJX8IF_EthernetOpen(self.device_id, self.eth_cfg)
        rospy.loginfo("LJX8IF_EthernetOpen: 0x%X", res)
        if res != 0:
            rospy.logfatal("Failed to connect controller")
            sys.exit(1)

        # optional Messung starten (falls Controller nicht extern getriggert wird)
        if self.start_measure:
            res = LJXAwrap.LJX8IF_StartMeasure(self.device_id)
            rospy.loginfo("LJX8IF_StartMeasure: 0x%X", res)

        # --- Request/Response/Info-Strukturen vorbereiten ---
        self.req = LJXAwrap.LJX8IF_GET_PROFILE_REQUEST()
        self.rsp = LJXAwrap.LJX8IF_GET_PROFILE_RESPONSE()
        self.info = LJXAwrap.LJX8IF_PROFILE_INFO()

        # Wie im Sample: von aktueller Position, 1 Profil, nichts löschen
        self.req.byTargetBank = 0x0      # aktive Bank
        self.req.byPositionMode = 0x0    # 0: from current position
        self.req.dwGetProfileNo = 0x0
        self.req.byGetProfileCount = 1
        self.req.byErase = 0

        # Buffergröße wie im Keyence-Sample berechnen
        header_size = ctypes.sizeof(LJXAwrap.LJX8IF_PROFILE_HEADER)
        footer_size = ctypes.sizeof(LJXAwrap.LJX8IF_PROFILE_FOOTER)
        dataSize = header_size + footer_size
        dataSize += ctypes.sizeof(ctypes.c_uint) * self.xpoint_num * (1 + self.with_lumi)
        dataSize *= self.req.byGetProfileCount

        self.data_size = ctypes.c_uint(dataSize)
        data_num_in_4byte = int(dataSize / ctypes.sizeof(ctypes.c_uint))
        self.profil_buf = (ctypes.c_int * data_num_in_4byte)()

        rospy.loginfo("Profile buffer: xpoints=%d, with_lumi=%d, bytes=%d",
                      self.xpoint_num, self.with_lumi, self.data_size.value)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        try:
            while not rospy.is_shutdown():
                self.read_and_publish_profile()
                rate.sleep()
        finally:
            # Aufräumen
            if self.start_measure:
                LJXAwrap.LJX8IF_StopMeasure(self.device_id)
            LJXAwrap.LJX8IF_CommunicationClose(self.device_id)

    def read_and_publish_profile(self):
        # Profil abfragen
        res = LJXAwrap.LJX8IF_GetProfile(
            self.device_id,
            ctypes.byref(self.req),
            ctypes.byref(self.rsp),
            ctypes.byref(self.info),
            self.profil_buf,
            self.data_size,
        )

        if res != 0:
            rospy.logwarn_throttle(1.0, "LJX8IF_GetProfile error: 0x%X", res)
            return

        x_count = self.info.wProfileDataCount
        if x_count <= 0 or x_count > self.xpoint_num:
            rospy.logwarn_throttle(1.0,
                                   "Unexpected wProfileDataCount=%d (xpoint_num=%d)",
                                   x_count, self.xpoint_num)
            return

        # Offsets (wie im Keyence-Sample)
        header_size = ctypes.sizeof(LJXAwrap.LJX8IF_PROFILE_HEADER)
        addressOffset_height = int(header_size / ctypes.sizeof(ctypes.c_uint))
        addressOffset_lumi = addressOffset_height + x_count  # falls Luminanz später nötig

        # ---- 1) Float32MultiArray (Z in mm) ----
        z_vals_mm = []
        for i in range(x_count):
            z_val = self.profil_buf[addressOffset_height + i]
            if z_val <= -2147483645:  # invalid value
                z_vals_mm.append(float("nan"))
            else:
                z_mm = z_val / 100.0   # 0.01 µm -> µm
                z_mm /= 1000.0         # µm -> mm
                z_vals_mm.append(z_mm)

        raw_msg = Float32MultiArray()
        raw_msg.data = z_vals_mm
        self.raw_pub.publish(raw_msg)

        # ---- 2) PointCloud2 für RViz (x,z in Metern) ----
        # X: aus lXStart / lXPitch (einheiten wie im Sample) -> zuerst mm, dann m
        # Z: aus z_vals_mm -> mm -> m
        points = []
        for i in range(x_count):
            # X in mm
            x_val = (self.info.lXStart + self.info.lXPitch * i) / 100.0  # µm
            x_val /= 1000.0  # mm
            # in Meter
            x_m = x_val / 1000.0

            # Z in m (falls NaN, einfach NaN lassen)
            z_mm = z_vals_mm[i]
            if z_mm != z_mm:  # NaN-Check
                z_m = float("nan")
            else:
                z_m = z_mm / 1000.0

            y_m = 0.0
            points.append((x_m, y_m, z_m))

        pc_msg = self._points_to_pointcloud2(points,
                                             frame_id=self.frame_id,
                                             stamp=rospy.Time.now())
        self.pc_pub.publish(pc_msg)

    @staticmethod
    def _points_to_pointcloud2(points, frame_id="keyence_frame", stamp=None):
        """
        points: List[(x,y,z)] in Meter
        """
        pc = PointCloud2()
        if stamp is None:
            stamp = rospy.Time.now()

        pc.header = std_msgs.msg.Header()
        pc.header.stamp = stamp
        pc.header.frame_id = frame_id

        pc.height = 1
        pc.width = len(points)
        pc.is_bigendian = False
        pc.is_dense = False

        pc.fields = [
            PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        pc.point_step = 12
        pc.row_step = pc.point_step * pc.width

        import struct
        buff = []
        for x, y, z in points:
            buff.append(struct.pack("fff", float(x), float(y), float(z)))
        pc.data = b"".join(buff)

        return pc


if __name__ == "__main__":
    try:
        node = KeyenceProfileNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logfatal("Exception in keyence_ljx_profile_node: %s", e)
        sys.exit(1)
