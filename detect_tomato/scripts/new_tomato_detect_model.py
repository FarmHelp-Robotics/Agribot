#!/usr/bin/env python2

import rospy
import cv2
import numpy as np
import requests
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import BoundingBox2D

class TomatoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.detection_pub = rospy.Publisher("/tomato_detection", BoundingBox2D, queue_size=10)
        print("bro")
        self.api_url = rospy.get_param('~api_url', 'http://192.168.21.202:8080/video_feed')

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Convert the image to bytes
        _, img_encoded = cv2.imencode('.jpg', cv_image)
        img_bytes = img_encoded.tobytes()

        headers = {
            'Content-Type': 'application/octet-stream',
            'Frame-Width': str(cv_image.shape[1]),
            'Frame-Height': str(cv_image.shape[0]),
            'Client-Timestamp': str(time.time())
        }

        start = time.time()

        # Send the image to the server
        response = requests.post(self.api_url, data=img_bytes, headers=headers)
        end_time = time.time()

        if response.status_code == 200:
            response_data = response.json()
            detections = response_data.get('detections', [])
            tp = response_data.get('total-process', '')
            bi = response_data.get('Before-inference-time', '')
            ai = response_data.get('After-inference-time', '')
            tp_st = response_data.get('total-process-start-time', '')
            tp_et = response_data.get('total-process-end-time', '')
            rst = response_data.get('resizing-time', '')
            inference = response_data.get('Processing-Delay', '')

            rtt = end_time - start
            network_delay = rtt - float(tp) if tp else 0

            # Process detections
            for detection in detections:
                x1 = detection['x1']
                y1 = detection['y1']
                x2 = detection['x2']
                y2 = detection['y2']
                w = x2-x1
                h = y2-y1

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
                cv_image_copy = cv_image.copy()
                cv2.rectangle(cv_image_copy, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(cv_image_copy, (center_x, center_y), 5, (255, 0, 0), -1)
                cv2.imshow("Tomato Detection", cv_image_copy)
                cv2.waitKey(1)

        else:
            #rospy.logerr(f"API request failed with status code: {response.status_code}")
            print(response.status_code)

def main():
    rospy.init_node('tomato_detector', anonymous=True)
    print("hi")
    td = TomatoDetector()
    try:
        print("hi again")
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

