#!/usr/bin/env python3

import rospy
import rospkg
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import BoundingBox2D
import torch
import os
import onnxruntime as ort

class TomatoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.ort_session =  ort.InferenceSession(os.path.join(rospkg.RosPack().get_path("detetect_tomato"), "models/best_yolov8n_leaf.onnx"), providers=["CUDAExecutionProvider"])
        self.model = torch.load(os.path.join(rospkg.RosPack().get_path("detect_tomato"), "models/best_yolov8n_leaf.pt"))
        image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_raw')
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.detection_pub = rospy.Publisher("/tomato_detection", BoundingBox2D, queue_size=10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Perform inference using YOLO model
        results = self.model(cv_image)
        detections = []
        for result in results:
            for obj in result.boxes:
                label = result.names[int(obj.cls[0])]
                confidence = obj.conf[0]
                # Get bounding box coordinates
                x1, y1, x2, y2 = obj.xyxy[0].cpu().numpy()
                detection = {
                    'label': label,
                    'confidence': float(confidence),
                    'x1': float(x1),
                    'y1': float(y1),
                    'x2': float(x2),
                    'y2': float(y2)
                }
                detections.append(detection)
                w = x2 - x1
                h = y2 - y1
                center_x = x1 + w // 2
                center_y = y1 + h // 2

                # Create and publish the bounding box message
                bounding_box_msg = BoundingBox2D()
                bounding_box_msg.center.x = center_x
                bounding_box_msg.center.y = center_y
                bounding_box_msg.size_x = w
                bounding_box_msg.size_y = h
                self.detection_pub.publish(bounding_box_msg)
                # Optionally, display the image for debugging
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(cv_image, (int(center_x), int(center_y)), 5, (255, 0, 0), -1)
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
