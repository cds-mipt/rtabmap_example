#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import sys


def odom_received(odom):
    global tfBroadcaster
    global tfBuffer
    global odom_publisher
    global drift_factor
    global counter
    
    odom.pose.pose.position.y += counter * drift_factor
    counter += 1

    odom_to_base_link = geometry_msgs.msg.TransformStamped()
    odom_to_base_link.header.stamp = odom.header.stamp
    odom_to_base_link.header.frame_id = 'odom'
    odom_to_base_link.child_frame_id = 'base_link'
    odom_to_base_link.transform.translation = odom.pose.pose.position
    odom_to_base_link.transform.rotation = odom.pose.pose.orientation
    tfBroadcaster.sendTransform(odom_to_base_link)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise ValueError()
    drift_factor = float(sys.argv[1])
    counter = 0
    rospy.init_node('openvslam_odom_to_base_link')
    tfBroadcaster = tf2_ros.TransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    rospy.Subscriber('/odometry', Odometry, odom_received)
    rospy.spin()
    
