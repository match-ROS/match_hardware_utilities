#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions

from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState


class keyence_transform():
    def __init__(self):
        rospy.init_node('keyence_transform_node')
        rospy.Subscriber("/franka_state_controller/franka_states",FrankaState, self.pose_cb)   
        rospy.spin()

        

    def pose_cb(self,data):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "sensor_optical_frame"

        t.transform.translation.x = data.O_T_EE[12]
        t.transform.translation.y = data.O_T_EE[13]
        t.transform.translation.z = data.O_T_EE[14]
        T = [[data.O_T_EE[0],data.O_T_EE[4],data.O_T_EE[8],data.O_T_EE[12]],
            [data.O_T_EE[1],data.O_T_EE[5],data.O_T_EE[9],data.O_T_EE[13]],
            [data.O_T_EE[2],data.O_T_EE[6],data.O_T_EE[10],data.O_T_EE[14]],
            [data.O_T_EE[3],data.O_T_EE[7],data.O_T_EE[11],data.O_T_EE[15]]]

        q = tf_conversions.transformations.quaternion_from_matrix(T)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)


if __name__=="__main__":
    keyence_transform()