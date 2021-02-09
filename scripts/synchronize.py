#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
import tf2_ros
import geometry_msgs.msg


def odom_received(odom):
    global odom_pub
    odom.header.stamp = rospy.get_rostime()
    odom_pub.publish(odom)
    
    
def vel_points_received(vel_points):
    global vel_points_pub
    vel_points.header.stamp = rospy.get_rostime()
    vel_points_pub.publish(vel_points)
    
    
def left_image_received(image):
    global left_image_pub
    image.header.stamp = rospy.get_rostime()
    left_image_pub.publish(image)


def left_image_gray_received(image):
    global left_image_gray_pub
    image.header.stamp = rospy.get_rostime()
    left_image_gray_pub.publish(image)
    
    
def left_camera_info_received(cam_info):
    global left_cam_info_pub
    cam_info.header.stamp = rospy.get_rostime()
    left_cam_info_pub.publish(cam_info)
    
    
def right_image_received(image):
    global right_image_pub
    image.header.stamp = rospy.get_rostime()
    right_image_pub.publish(image)
    
    
def right_camera_info_received(cam_info):
    global right_cam_info_pub
    cam_info.header.stamp = rospy.get_rostime()
    right_cam_info_pub.publish(cam_info)
    
    
def depth_image_received(image):
    global depth_image_pub
    image.header.stamp = rospy.get_rostime()
    depth_image_pub.publish(image)
    
    
def depth_camera_info_received(cam_info):
    global depth_cam_info_pub
    cam_info.header.stamp = rospy.get_rostime()
    depth_cam_info_pub.publish(cam_info)


if __name__ == '__main__':
    rospy.init_node('synchronize')
    
    rospy.Subscriber('/OpenVSLAM/odom', Odometry, odom_received)
    odom_pub = rospy.Publisher('/synchronized/odom', Odometry, queue_size=10)
    
    rospy.Subscriber('/velodyne_points', PointCloud2, vel_points_received)
    vel_points_pub = rospy.Publisher('/synchronized/velodyne_points', PointCloud2, queue_size=10)
    
    rospy.Subscriber('/zed_node/left/image_rect_color', Image, left_image_received)
    left_image_pub = rospy.Publisher('/synchronized/left_image', Image, queue_size=10)
    rospy.Subscriber('/zed_node/left/image_rect_gray', Image, left_image_gray_received)
    left_image_gray_pub = rospy.Publisher('/synchronized/left_image_gray', Image, queue_size=10)
    rospy.Subscriber('/zed_node/left/camera_info', CameraInfo, left_camera_info_received)
    left_cam_info_pub = rospy.Publisher('/synchronized/left_camera_info', CameraInfo, queue_size=10)
    
    rospy.Subscriber('/zed_node/right/image_rect_color', Image, right_image_received)
    right_image_pub = rospy.Publisher('/synchronized/right_image', Image, queue_size=10)
    rospy.Subscriber('/zed_node/right/camera_info', CameraInfo, right_camera_info_received)
    right_cam_info_pub = rospy.Publisher('/synchronized/right_camera_info', CameraInfo, queue_size=10)
    
    rospy.Subscriber('/zed_node/depth/depth_registered', Image, depth_image_received)
    depth_image_pub = rospy.Publisher('/synchronized/depth_image', Image, queue_size=10)
    rospy.Subscriber('/zed_node/depth/camera_info', CameraInfo, depth_camera_info_received)
    depth_cam_info_pub = rospy.Publisher('/synchronized/depth_camera_info', CameraInfo, queue_size=10)
    
    rospy.spin()

