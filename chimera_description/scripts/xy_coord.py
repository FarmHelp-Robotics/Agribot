#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2

# Global variable to store the clicked coordinates
clicked_point = (-1, -1)

def mouse_callback(event, x, y, flags, param):
    global clicked_point
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_point = (x, y)
        #rospy.loginfo("Clicked coordinates: {}",format(clicked_point))

def image_callback(msg):
    global clicked_point
    
    # Convert ROS image message to OpenCV image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    # Display the image with OpenCV
    cv2.imshow("Camera Image", cv_image)
    cv2.setMouseCallback("Camera Image", mouse_callback)

    # Wait for a key event to process OpenCV window updates
    cv2.waitKey(1)

def main():
    rospy.init_node('mouse_click_listener', anonymous=True)
    
    # Publisher for the xy coordinates
    coord_pub = rospy.Publisher('/xy_coord', Float64MultiArray, queue_size=10)
    
    # Subscribe to the camera image topic
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    
    # Rate to publish coordinates every second
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        if clicked_point != (-1, -1):
            # Create a Float64MultiArray message
            coord_msg = Float64MultiArray()
            coord_msg.data = [float(clicked_point[0]), float(clicked_point[1])]
            
            # Publish the coordinates
            coord_pub.publish(coord_msg)
            
            rospy.loginfo("Published coordinates: {}".format(clicked_point))

        
        rate.sleep()

    # Cleanup OpenCV windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

