#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tf
import tf2_ros

# Global variables for HSV thresholds
hsv_lower = np.array([0, 100, 100])  # Example lower HSV threshold red
hsv_upper = np.array([10, 255, 255])  # Example upper HSV threshold

#hsv_lower = np.array([105, 55, 54])  # Example lower HSV threshold volet marker
#hsv_upper = np.array([118, 140, 138])  # Example upper HSV threshold


class TomatoTracker:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('xyz_coord', anonymous=True)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Initialize TF broadcaster and listener
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize Publisher
        self.tomato_pose_pub = rospy.Publisher("/tomato_pose", Pose, queue_size=10)
        self.coord_pub = rospy.Publisher('/xy_coord', Float64MultiArray, queue_size=10)

        # Subscribe to camera topics
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.camera_info_callback)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_image_callback)
        rospy.Subscriber("/xy_coord", Float64MultiArray, self.xy_coord_callback)

        # Placeholder for intrinsics and pixel coordinates
        self.intrinsics = None
        self.pixel = None

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to HSV and create a mask
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, hsv_lower, hsv_upper)

        # Find contours
        contour_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contour_info[0] if len(contour_info) == 2 else contour_info[1]
        hierarchy = contour_info[1] if len(contour_info) == 2 else None
        
        if contours:
            # Find the largest contour and draw it
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # Publish the center coordinates
                coord_msg = Float64MultiArray()
                coord_msg.data = [float(cX), float(cY)]
                self.coord_pub.publish(coord_msg)

                # Draw the center of the contour
                cv2.circle(cv_image, (cX, cY), 5, (0, 0, 255), -1)
                #rospy.loginfo("Center of contour: ({}, {})".format(cX, cY))

        # Display the image with contours (optional)
        #cv2.imshow("Camera Image", cv_image)
        cv2.waitKey(1)

    def camera_info_callback(self, camera_info):
        # Extract intrinsics from the CameraInfo message
        self.intrinsics = {
            'fx': camera_info.K[0],
            'fy': camera_info.K[4],
            'cx': camera_info.K[2],
            'cy': camera_info.K[5]
        }

    def xy_coord_callback(self, data):
        # Extract the x and y coordinates from the Float64MultiArray message
        if len(data.data) >= 2:
            self.pixel = [int(data.data[0]), int(data.data[1])]
        else:
            rospy.logwarn("Received an invalid xy coordinate")

    def depth_image_callback(self, depth_image):
        if self.intrinsics is None:
            rospy.logwarn("Waiting for camera intrinsics...")
            return

        if self.pixel is None:
            rospy.logwarn("Waiting for pixel coordinates...")
            return

        try:
            # Convert the ROS Image message to OpenCV format
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
            r_x, r_y = self.pixel

            # Get depth at the specified pixel (in millimeters)
            r_z_mm = cv_depth_image[r_y, r_x]

            # Convert 2D to 3D using the intrinsics and depth value
            point_3d = self.deproject_pixel_to_point([r_x, r_y], r_z_mm)

            # Check if the point is valid
            if point_3d == [-0.0, -0.0, 0.0]:
                #rospy.loginfo("Skipping invalid point: {}".format(point_3d))
                return

            rate = rospy.Rate(10.0)

            # Send transform for tomato pose relative to camera
            self.tf_broadcaster.sendTransform(
                (point_3d[0], point_3d[1], point_3d[2]),
                tf.transformations.quaternion_from_euler(0, np.pi / 2, 0),
                rospy.Time.now(),
                'red_pose',
                'camera_depth_optical_frame'
            )
            
            rate.sleep()

            tom_pose = self.tf_buffer.lookup_transform("base_footprint", "red_pose", rospy.Time(0),rospy.Duration(1.0))
           
            # Extract translation and rotation values
            t_x = tom_pose.transform.translation.x
            t_y = tom_pose.transform.translation.y
            t_z = tom_pose.transform.translation.z
            t_roll = tom_pose.transform.rotation.x
            t_pitch = tom_pose.transform.rotation.y
            t_yaw = tom_pose.transform.rotation.z
            t_w = tom_pose.transform.rotation.w

            # Update and publish the tomato pose
            pub_tom_pose = Pose()
            pub_tom_pose.position.x = t_x
            pub_tom_pose.position.y = t_y
            pub_tom_pose.position.z = t_z
            pub_tom_pose.orientation.x = t_roll
            pub_tom_pose.orientation.y = t_pitch
            pub_tom_pose.orientation.z = t_yaw
            pub_tom_pose.orientation.w = t_w

            # if distance is less that specify condition 
            self.tf_broadcaster.sendTransform(
                (t_x, t_y, t_z),
                tf.transformations.quaternion_from_euler(0, np.pi / 2, 0),
                rospy.Time.now(),
                'tomato_pose',
                'base_footprint'
            )
            
            self.tomato_pose_pub.publish(pub_tom_pose)
            rate.sleep()
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform error: %s" % e)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
        except IndexError:
            rospy.logerr("Pixel coordinates out of depth image bounds.")


    def deproject_pixel_to_point(self, pixel, depth_mm):
        # Convert depth from millimeters to meters
        depth_m = depth_mm / 1000.0

        # Extract intrinsics
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        cx = self.intrinsics['cx']
        cy = self.intrinsics['cy']

        # Get the 2D pixel coordinates
        r_x, r_y = pixel

        # Deproject to 3D space using the depth in meters
        X = (r_x - cx) * depth_m / fx
        Y = (r_y - cy) * depth_m / fy
        Z = depth_m

        return [X, Y, Z]

if __name__ == '__main__':
    try:
        # Instantiate the TomatoTracker class and spin ROS
        tracker = TomatoTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

