#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from tf2_geometry_msgs import PointStamped
from vision_msgs.msg import BoundingBox2D
import tf2_ros
from tf.transformations import *

class DepthSelector:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image = None
        self.camera_image = None
        self.camera_matrix = None
	self.tf_buffer = tf2_ros.Buffer()
	self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # TODO: Add parameters to get the topics
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        #rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        rospy.Subscriber("/tomato_detection", BoundingBox2D, self.tomato_detection_callback)

        self.selected_point = None
        data = rospy.wait_for_message('/camera/depth/camera_info', CameraInfo, timeout=5)
        self.camera_matrix = data.K
        self.pose_pub = rospy.Publisher('/the_target_pos', PointStamped, queue_size=10)  # Initialize the publisher

    def depth_callback(self, data):
        try:
		self.depth_image = self.bridge.imgmsg_to_cv2(data)
	except CvBridgeError as e:
		rospy.logerr("CvBridge Error: {0}".format(e))

    def tomato_detection_callback(self, data):
        # Extract the center of the bounding box
        x_center = int(data.center.x)
        y_center = int(data.center.y)
        try:
 
                Z = self.depth_image[y_center,x_center]
	        if Z <=0.0: return 

                Z *= 0.001 # TODO: conversion from mm to m; a way to do this automatically?
                
	        pos = PointStamped()
	        pos.header.stamp = rospy.Time(0)
	        pos.header.frame_id = "camera_depth_optical_frame" # TODO: Make this a parameter?
	        X, Y = self.data_transformation(x_center, y_center, Z)

	        pos.point.x = X
	        pos.point.y = Y
                pos.point.z = Z
	
	        self.pose_pub.publish(pos)

	except Exception as e:
		rospy.loginfo("Bro what? 0_0 {}".format(e))

    def data_transformation(self, x, y, depth):
        # Transforming 2D x and y to 3D x and y using the camera_info matrix
        x_new = depth * ((x - self.camera_matrix[2]) / self.camera_matrix[0])
        y_new = depth * ((y - self.camera_matrix[5]) / self.camera_matrix[4])
        return (x_new, y_new)
    
    def cleanup(self):
        rospy.loginfo("Shutting down the depth node")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('depth_selector', anonymous=True)
    ds = DepthSelector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

