#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import ctypes
import sys

import rospy
from std_msgs.msg import Float32MultiArray

import LJXAwrap  # liegt bei dir schon neben den Samples


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

        self.pub = rospy.Publisher("/profiles", Float32MultiArray, queue_size=1)

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
        data_size = header_size + footer_size
        data_size += ctypes.sizeof(ctypes.c_uint) * self.xpoint_num * (1 + self.with_lumi)
        data_size *= self.req.byGetProfileCount

        self.data_size = ctypes.c_uint(data_size)
        data_num_in_4byte = int(data_size / ctypes.sizeof(ctypes.c_uint))
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
        offset_height = int(header_size / ctypes.sizeof(ctypes.c_uint))
        # offset_lumi = offset_height + x_count  # falls du später Luminanz brauchst

        # Z-Werte in mm (gleiche Skalierung wie sample_HowToCallFunctions)
        z_vals_mm = []
        for i in range(x_count):
            z_val = self.profil_buf[offset_height + i]

            # Ungültige Werte (siehe Sample)
            if z_val <= -2147483645:
                z_vals_mm.append(float("nan"))
            else:
                z_mm = z_val / 100.0  # 0.01 µm -> µm
                z_mm /= 1000.0        # µm -> mm
                z_vals_mm.append(z_mm)

        msg = Float32MultiArray()
        msg.data = z_vals_mm
        self.pub.publish(msg)


if __name__ == "__main__":
    try:
        node = KeyenceProfileNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logfatal("Exception in keyence_ljx_profile_node: %s", e)
        sys.exit(1)
