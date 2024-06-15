#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2 
import numpy as np 

class DepthSelector:
	def __init__(self):
		self.bridge = CvBridge()
		self.depth_image = None
		self.camera_image = None 
		self.camera_matrix = None
		
		rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
		rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
                # ospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.camera_info_callback)

		cv2.namedWindow("Camera Image")
		cv2.setMouseCallback("Camera Image", self.mouse_callback)
                
		self.selected_point = None 
	        data = rospy.wait_for_message('/camera/depth/camera_info', CameraInfo, timeout=5)
                self.camera_matrix = data.K


	def depth_callback(self, data):
		try:
			self.depth_image = self.bridge.imgmsg_to_cv2(data)
		except CvBridgeError as e:
			rospy.logerr("cvbridge errorrrr 0_0 :{0}".format(e))
	def camera_callback(self, data):
		try:
			self.camera_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			if self.camera_image is not None:
				if self.selected_point is not None and self.depth_image is not None:
					x, y = self.selected_point 
					Z = self.depth_image[y, x]
					cv2.circle(self.camera_image, (x, y), 5, (0, 0, 255), -1 )
					print(x,y)
					#calling the data transformation function
					X, Y = self.data_transformation(x, y, Z)
					print(X,Y)
					cv2.putText(self.camera_image, "(z) = ({:.2f}mm)".format(Z), (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
				cv2.imshow("Camera Image", self.camera_image)
				cv2.waitKey(1)
		except CvBridgeError as e:
			rospy.logerr("CvBridge Error: {0}".format(e))
                except AttributeError as e:
                    rospy.logerr("Waiting for data")

    	#def camera_info_callback(self, data):
	#	self.camera_matrix = data.K
	#	print(self.camera_matrix)
        #	rospy.loginfo("Camera intrinsic matrix: \n{}".format(self.camera_matrix))
	
	def data_transformation(self, x, y, depth):
		#transforming 2d x and y to 3d x and y using the camera_info matrix
		x_new = depth*((x - self.camera_matrix[2])/self.camera_matrix[0])
		y_new = depth*((y - self.camera_matrix[5])/self.camera_matrix[4])
		return (x_new, y_new)
	
	def mouse_callback(self, event, x, y, flags, param):
		if event == cv2.EVENT_LBUTTONDOWN:
			self.selected_point = (x, y)
	def cleanup(self):
		rospy.loginfo("Shutting down da depth node:")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.init_node('depth_selector', anonymous = True)
	ds = DepthSelector()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")
	cv2.destroyAllWindows()
