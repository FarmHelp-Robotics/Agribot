#!/usr/bin/env python2

import rospy
import cv2
import numpy as np
import requests
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import BoundingBox2D
import threading

class TomatoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.detection_pub = rospy.Publisher("/tomato_detection", BoundingBox2D, queue_size=10)
        self.api_url = rospy.get_param('~api_url', 'http://192.168.21.202:8080/video_feed')
        self.stream_url = rospy.get_param('~stream_url', 'http://192.168.21.202:8080/stream')
        threading.Thread(target=self.display_stream).start()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        _, img_encoded = cv2.imencode('.jpg', cv_image)
        img_bytes = img_encoded.tostring()

        headers = {
            'Content-Type': 'application/octet-stream',
            'Frame-Width': str(cv_image.shape[1]),
            'Frame-Height': str(cv_image.shape[0]),
            'Client-Timestamp': str(time.time())
        }

        response = requests.post(self.api_url, data=img_bytes, headers=headers)
        if response.status_code != 200:
            rospy.logerr("API request failed with status code: {}".format(response.status_code))

    def display_stream(self):
        stream = requests.get(self.stream_url, stream=True)
        if stream.status_code == 200:
            bytes_data = bytes()
            for chunk in stream.iter_content(chunk_size=1024):
                bytes_data += chunk
                a = bytes_data.find(b'\xff\xd8')
                b = bytes_data.find(b'\xff\xd9')
                if a != -1 and b != -1:
                    jpg = bytes_data[a:b + 2]
                    bytes_data = bytes_data[b + 2:]
                    img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if img is not None:
                        cv2.imshow('Stream', img)
                        cv2.waitKey(1)
        else:
            rospy.logerr("Stream request failed with status code: {}".format(stream.status_code))

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

