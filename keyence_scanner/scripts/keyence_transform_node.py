#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions
import math

from geometry_msgs.msg import PoseStamped
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header

#import roslib; roslib.load_manifest('laser_assembler')


class KeyenceTransformNode():
    def __init__(self):
        rospy.init_node('keyence_transform_node')

        self.tcp_scanner_offset = [0.038,0.038,0.7364]

        self.ur_pose_topic = rospy.get_param("~ur_pose_topic", "/mur620c/UR10_r/ur_calibrated_pose")

        rospy.Subscriber(self.ur_pose_topic,PoseStamped, self.pose_cb) 
        rospy.loginfo("waiting for pose message")
        rospy.wait_for_message(self.ur_pose_topic,PoseStamped)
        
        rospy.Subscriber("/profiles",PointCloud2, self.pointcloud_cb)   
        self.cloud_pub = rospy.Publisher("/cloud_out",PointCloud2, queue_size = 10)
        self.cloud_out = PointCloud2()
        
        
        
        
        rospy.loginfo("Transform publisher running")
        rospy.spin()

        

    def pose_cb(self,data = PoseStamped()):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        # calculate transformation from world to EE
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "mur620c/UR10_r/base_link"
        t.child_frame_id = "sensor_optical_frame"

        R = tf_conversions.transformations.quaternion_matrix([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])

        t.transform.translation.x = -data.pose.position.x + self.tcp_scanner_offset[0] * R[0,0] + self.tcp_scanner_offset[1] * R[0,1] 
        t.transform.translation.y = -data.pose.position.y + self.tcp_scanner_offset[1] * R[1,1] + self.tcp_scanner_offset[2] * R[1,2]
        t.transform.translation.z = data.pose.position.z - self.tcp_scanner_offset[2]
        
        # the data is rotated by 180 degrees around the z axis
        q = tf_conversions.transformations.quaternion_from_euler(math.pi,0,0.995-math.pi)
        
        # invert quaternion
        q_inv = [data.pose.orientation.x,-data.pose.orientation.y,-data.pose.orientation.z,-data.pose.orientation.w]
        
        q_transform = tf_conversions.transformations.quaternion_multiply(q,q_inv)
        
        t.transform.rotation.x = q_transform[0]
        t.transform.rotation.y = q_transform[1]
        t.transform.rotation.z = q_transform[2]
        t.transform.rotation.w = q_transform[3]
        self.transform = t

        br.sendTransform(t)

    def pointcloud_cb(self,cloud):
        self.cloud_out = do_transform_cloud(cloud, self.transform)
        self.cloud_pub.publish(self.cloud_out)

        
        
                
                





if __name__=="__main__":
    KeyenceTransformNode()