#!/usr/bin/env python2

import rospy
import tf2_ros
import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg
from cv_bridge import CvBridge

# Updated camera intrinsic parameters
fx = 430.734619140625  # Focal length in x direction (in pixels)
fy = 430.734619140625  # Focal length in y direction (in pixels)
cx = 422.6560974121094  # Principal point in x direction (in pixels)
cy = 238.5064239501953  # Principal point in y direction (in pixels)

clicked_point = (-1, -1)
depth_image = None

# Create a TF2 broadcaster
tf_broadcaster = tf2_ros.TransformBroadcaster()

def xy_coord_callback(msg):
    global clicked_point
    if msg.data:
        clicked_point = (int(msg.data[0]), int(msg.data[1]))
        #rospy.loginfo("Received coordinates: ({}, {})".format(clicked_point[0], clicked_point[1]))
    else:
        rospy.logwarn("Received empty coordinate data.")

def depth_image_callback(msg):
    global depth_image
    bridge = CvBridge()
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        #rospy.loginfo("Received depth image of size: {} x {}".format(depth_image.shape[1], depth_image.shape[0]))
    except Exception as e:
        rospy.logerr("Failed to convert depth image: {}".format(e))

def main():
    rospy.init_node('depth_value_converter', anonymous=True)
    pose_pub = rospy.Publisher('/red_pose', geometry_msgs.msg.PoseStamped, queue_size=10)
    
    rospy.Subscriber('/xy_coord', std_msgs.msg.Float64MultiArray, xy_coord_callback)
    rospy.Subscriber('/camera/depth/image_rect_raw', sensor_msgs.msg.Image, depth_image_callback)
    
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        if clicked_point != (-1, -1):
            #rospy.loginfo("Processing coordinates: ({}, {})".format(clicked_point[0], clicked_point[1]))
            
            if depth_image is not None:
                x, y = clicked_point

                # Ensure the coordinates are within the image bounds
                if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
                    depth_value = depth_image[y, x] # dept value in mm 
                    
                    rospy.loginfo("Depth value at ({}, {}) in cm: {:.2f}".format(x, y, depth_value))

                    # Convert depth value to real-world coordinates
                    z = depth_value/1000.0  # dept value in m  
                    X = (x - cx) * z / fx
                    Y = (y - cy) * z / fy

                    # Create and publish pose message
                    pose_msg = geometry_msgs.msg.PoseStamped()
                    pose_msg.header.stamp = rospy.Time.now()
                    pose_msg.header.frame_id = "camera_link"  # Frame ID of the base frame
                    pose_msg.pose.position.x = X
                    pose_msg.pose.position.y = Y
                    pose_msg.pose.position.z = z
                    
                    pose_pub.publish(pose_msg)
                    
                    rospy.loginfo("Red_pose: X = {:.4f} , Y = {:.4f} , Z = {:.4f} ".format(X , Y, z))
                    
                    # Broadcast transformation from "camera_link" to "red_pose"
                    transform = geometry_msgs.msg.TransformStamped()
                    transform.header.stamp = rospy.Time.now()
                    transform.header.frame_id = "camera_link"
                    transform.child_frame_id = "red_pose"
                    transform.transform.translation.x = X
                    transform.transform.translation.y = Y
                    transform.transform.translation.z = z
                    transform.transform.rotation.x = 0.0
                    transform.transform.rotation.y = 0.0
                    transform.transform.rotation.z = 0.0
                    transform.transform.rotation.w = 1.0  # Identity quaternion
                    
                    tf_broadcaster.sendTransform(transform)

                else:
                    rospy.logwarn("Clicked point is out of bounds: ({}, {})".format(x, y))
            else:
                rospy.logwarn("Depth image not yet received.")
        
        rate.sleep()

if __name__ == '__main__':
    main()

