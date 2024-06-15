#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import BoundingBox2D

class TomatoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_raw')
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.detection_pub = rospy.Publisher("/tomato_detection", BoundingBox2D, queue_size=10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Convert the image to the HSV color space
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the range of red color in HSV
        lower_red = np.array([10, 100, 20])
        upper_red = np.array([25, 255, 255])

        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours in the mask
        try:
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        except ValueError:
            _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # Calculate the center of the bounding box
            center_x = x + w // 2
            center_y = y + h // 2
            
            # Create and publish the bounding box message
            bounding_box_msg = BoundingBox2D()
            bounding_box_msg.center.x = center_x
            bounding_box_msg.center.y = center_y
            bounding_box_msg.size_x = w
            bounding_box_msg.size_y = h
            
            self.detection_pub.publish(bounding_box_msg)
            
            # Optionally, display the image for debugging
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(cv_image, (center_x, center_y), 5, (255, 0, 0), -1)
            cv2.imshow("Tomato Detection", cv_image)
            cv2.waitKey(1)

def main():
    rospy.init_node('tomato_detector', anonymous=True)
    td = TomatoDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
