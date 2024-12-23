#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Callback function for image subscriber
def image_callback(msg):
    global bridge
    global img

    # Convert ROS Image message to OpenCV image
    img = bridge.imgmsg_to_cv2(msg, "bgr8")

def nothing(x):
    pass

def main():
    global img
    global bridge
    img = None
    bridge = CvBridge()

    # Initialize the ROS node
    rospy.init_node('image_masking_node', anonymous=True)
    
    # Subscribe to the image topic
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

    # Create a window and trackbars for HSV values
    cv2.namedWindow('Masked Image')
    cv2.createTrackbar('H Min', 'Masked Image', 0, 179, nothing)
    cv2.createTrackbar('H Max', 'Masked Image', 179, 179, nothing)
    cv2.createTrackbar('S Min', 'Masked Image', 0, 255, nothing)
    cv2.createTrackbar('S Max', 'Masked Image', 255, 255, nothing)
    cv2.createTrackbar('V Min', 'Masked Image', 0, 255, nothing)
    cv2.createTrackbar('V Max', 'Masked Image', 255, 255, nothing)

    while not rospy.is_shutdown():
        if img is not None:
            # Get current positions of trackbars
            h_min = cv2.getTrackbarPos('H Min', 'Masked Image')
            h_max = cv2.getTrackbarPos('H Max', 'Masked Image')
            s_min = cv2.getTrackbarPos('S Min', 'Masked Image')
            s_max = cv2.getTrackbarPos('S Max', 'Masked Image')
            v_min = cv2.getTrackbarPos('V Min', 'Masked Image')
            v_max = cv2.getTrackbarPos('V Max', 'Masked Image')

            # Convert the image to HSV color space
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Create the mask based on the HSV values
            lower_hsv = np.array([h_min, s_min, v_min])
            upper_hsv = np.array([h_max, s_max, v_max])
            mask = cv2.inRange(hsv_img, lower_hsv, upper_hsv)

            # Apply the mask to the original image
            masked_img = cv2.bitwise_and(img, img, mask=mask)

            # Show the masked image
            cv2.imshow('Masked Image', masked_img)

        # Exit if the user presses 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

