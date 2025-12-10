#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import ctypes
import sys
import struct
import numpy as np

import rospy
import tf

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg

import LJXAwrap  # Keyence API wrapper


class KeyenceProfileNode(object):
    def __init__(self):
        rospy.init_node("keyence_ljx_profile_node")

        # TF
        self.tf_listener = tf.TransformListener()

        # --- Parameter ---
        self.device_id = rospy.get_param("~device_id", 0)
        ip_str = rospy.get_param("~ip_address", "192.168.12.88")
        self.port = rospy.get_param("~port", 24691)
        self.rate_hz = rospy.get_param("~rate", 40.0)
        self.xpoint_num = rospy.get_param("~xpoint_num", 3200)
        self.with_lumi = rospy.get_param("~with_luminance", 1)
        self.start_measure = rospy.get_param("~start_measure", False)
        self.frame_id = rospy.get_param("~frame_id", "keyence_frame")
        self.publish_profiles_in_map = rospy.get_param("~publish_profiles_in_map", True)
        self.map_frame = rospy.get_param("~map_frame", "map")

        # Optional Downsampling
        self.downsample = rospy.get_param("~downsample", 1)  # 1=no DS, 2=50%, 4=25%, etc

        # Publisher
        self.raw_pub = rospy.Publisher("/profiles_float", Float32MultiArray, queue_size=1)
        self.pc_pub = rospy.Publisher("/profiles", PointCloud2, queue_size=1)

        # --- Ethernet config ---
        self.eth_cfg = LJXAwrap.LJX8IF_ETHERNET_CONFIG()
        ip_parts = [int(x) for x in ip_str.split(".")]
        for i in range(4):
            self.eth_cfg.abyIpAddress[i] = ip_parts[i]
        self.eth_cfg.wPortNo = self.port

        # Connect
        res = LJXAwrap.LJX8IF_EthernetOpen(self.device_id, self.eth_cfg)
        rospy.loginfo("EthernetOpen: 0x%X", res)
        if res != 0:
            rospy.logfatal("Failed to connect controller")
            sys.exit(1)

        if self.start_measure:
            LJXAwrap.LJX8IF_StartMeasure(self.device_id)

        # Profile structures
        self.req = LJXAwrap.LJX8IF_GET_PROFILE_REQUEST()
        self.rsp = LJXAwrap.LJX8IF_GET_PROFILE_RESPONSE()
        self.info = LJXAwrap.LJX8IF_PROFILE_INFO()

        self.req.byTargetBank = 0
        self.req.byPositionMode = 0
        self.req.dwGetProfileNo = 0
        self.req.byGetProfileCount = 1
        self.req.byErase = 0

        # Buffer size
        header = ctypes.sizeof(LJXAwrap.LJX8IF_PROFILE_HEADER)
        footer = ctypes.sizeof(LJXAwrap.LJX8IF_PROFILE_FOOTER)
        datasize = header + footer
        datasize += ctypes.sizeof(ctypes.c_uint) * self.xpoint_num * (1 + self.with_lumi)
        self.data_size = ctypes.c_uint(datasize)

        n_ints = int(datasize / ctypes.sizeof(ctypes.c_uint))
        self.profil_buf = (ctypes.c_int * n_ints)()

    # ----------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        try:
            while not rospy.is_shutdown():
                self.read_and_publish_profile()
                rate.sleep()
        finally:
            if self.start_measure:
                LJXAwrap.LJX8IF_StopMeasure(self.device_id)
            LJXAwrap.LJX8IF_CommunicationClose(self.device_id)

    # ----------------------------------------------------------
    def read_and_publish_profile(self):
        # --- Keyence Call (fast) ---
        res = LJXAwrap.LJX8IF_GetProfile(
            self.device_id,
            ctypes.byref(self.req),
            ctypes.byref(self.rsp),
            ctypes.byref(self.info),
            self.profil_buf,
            self.data_size,
        )
        if res != 0:
            rospy.logwarn_throttle(1.0, "GetProfile error: 0x%X" % res)
            return

        x_count = self.info.wProfileDataCount

        # Parse height values (vectorized)
        header_size = ctypes.sizeof(LJXAwrap.LJX8IF_PROFILE_HEADER)
        offset_h = int(header_size / 4)

        raw_data = np.frombuffer(self.profil_buf, dtype=np.int32, count=x_count, offset=header_size)

        # invalid values → NaN
        z_mm = np.where(raw_data <= -2147483645, np.nan, raw_data.astype(np.float32) * 1e-6)

        # Publish raw float profile
        raw_msg = Float32MultiArray()
        raw_msg.data = z_mm.tolist()
        self.raw_pub.publish(raw_msg)

        # ------------------------------------------------------
        # Build 3D points (NumPy, vectorized)
        # ------------------------------------------------------
        # X (µm → m)
        xs = (self.info.lXStart + self.info.lXPitch * np.arange(x_count)) * 1e-6
        ys = np.zeros_like(xs)
        zs = z_mm * 1e-3  # mm → m

        # downsample
        if self.downsample > 1:
            xs = xs[::self.downsample]
            ys = ys[::self.downsample]
            zs = zs[::self.downsample]

        P = np.column_stack([xs, ys, zs])  # shape (N,3)

        # ------------------------------------------------------
        # Transform points into map via TF (NumPy, fast)
        # ------------------------------------------------------
        if self.publish_profiles_in_map:
            try:
                trans, rot = self.tf_listener.lookupTransform(self.map_frame, self.frame_id, rospy.Time(0))
                T = tf.transformations.quaternion_matrix(rot)
                T[:3, 3] = trans

                # Homogeneous transform
                P_h = np.hstack([P, np.ones((P.shape[0], 1))])
                P = (T @ P_h.T).T[:, :3]

                frame_out = self.map_frame
            except Exception:
                frame_out = self.frame_id
        else:
            frame_out = self.frame_id

        # ------------------------------------------------------
        # Build PointCloud2 (one fast pack)
        # ------------------------------------------------------
        stamp = rospy.Time.now()
        pc_msg = PointCloud2()
        pc_msg.header = std_msgs.msg.Header(stamp=stamp, frame_id=frame_out)

        pc_msg.height = 1
        pc_msg.width = len(P)
        pc_msg.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
        ]
        pc_msg.is_bigendian = False
        pc_msg.point_step = 12
        pc_msg.row_step = pc_msg.point_step * pc_msg.width
        pc_msg.is_dense = False

        # struct.pack in einem einzigen Schritt → extrem schnell
        pc_msg.data = struct.pack("<%df" % (P.size), *P.flatten())

        self.pc_pub.publish(pc_msg)

    # ----------------------------------------------------------


if __name__ == "__main__":
    try:
        node = KeyenceProfileNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logfatal("Exception: %s", e)
        sys.exit(1)
