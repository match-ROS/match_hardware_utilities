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

        rospy.Subscriber("/mur620c/UR10_r/ur_calibrated_pose",PoseStamped, self.pose_cb)   
        rospy.sleep(1)
        rospy.Subscriber("/profiles",PointCloud2, self.pointcloud_cb)   
        self.cloud_pub = rospy.Publisher("/cloud_out",PointCloud2, queue_size = 10)
        self.cloud_out = PointCloud2()
        
        self.tcp_scanner_offset = [0.0,0.0,0.0]
        
        
        rospy.loginfo("Transform publisher running")
        rospy.spin()

        

    def pose_cb(self,data = PoseStamped()):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        # calculate transformation from world to EE
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "mur620c/UR10_r/base_link"
        t.child_frame_id = "sensor_optical_frame"

        t.transform.translation.x = -data.pose.position.x
        t.transform.translation.y = -data.pose.position.y
        t.transform.translation.z = data.pose.position.z
        t.transform.rotation.x = data.pose.orientation.x
        t.transform.rotation.y = data.pose.orientation.y
        t.transform.rotation.z = data.pose.orientation.z
        t.transform.rotation.w = data.pose.orientation.w
        self.transform = t

        br.sendTransform(t)

    def pointcloud_cb(self,cloud):
        self.cloud_out = do_transform_cloud(cloud, self.transform)
        self.cloud_pub.publish(self.cloud_out)

        
        
                
                





if __name__=="__main__":
    KeyenceTransformNode()