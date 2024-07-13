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
        self.model = cv2.dnn.readNetFromONNX('models/best_yolov8n_leaf.onnx')
        self.image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_raw')
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.detection_pub = rospy.Publisher("/tomato_detection", BoundingBox2D, queue_size=10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Preprocess the image
        blob = cv2.dnn.blobFromImage(cv_image, 1.0 / 255.0, (640, 640), (0, 0, 0), swapRB=True, crop=False)
        self.model.setInput(blob)
        outputs = self.model.forward(self.get_outputs_names(self.model))

        # Post-process the outputs
        for detection in self.process_detections(outputs, cv_image.shape):
            # Create and publish the bounding box message
            bounding_box_msg = BoundingBox2D()
            bounding_box_msg.center.x = detection['center_x']
            bounding_box_msg.center.y = detection['center_y']
            bounding_box_msg.size_x = detection['width']
            bounding_box_msg.size_y = detection['height']
            self.detection_pub.publish(bounding_box_msg)
            # Optionally, display the image for debugging
            cv2.rectangle(cv_image, (int(detection['x1']), int(detection['y1'])), 
                          (int(detection['x2']), int(detection['y2'])), (0, 255, 0), 2)
            cv2.circle(cv_image, (int(detection['center_x']), int(detection['center_y'])), 5, (255, 0, 0), -1)
        
        cv2.imshow("Tomato Detection", cv_image)
        cv2.waitKey(1)

    def get_outputs_names(self, net):
        layers_names = net.getLayerNames()
        return [layers_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    def process_detections(self, outputs, image_shape):
        detections = []
        height, width = image_shape[:2]
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x1 = center_x - w / 2
                    y1 = center_y - h / 2
                    x2 = center_x + w / 2
                    y2 = center_y + h / 2
                    detections.append({
                        'label': str(class_id),
                        'confidence': float(confidence),
                        'x1': float(x1),
                        'y1': float(y1),
                        'x2': float(x2),
                        'y2': float(y2),
                        'center_x': float(center_x),
                        'center_y': float(center_y),
                        'width': float(w),
                        'height': float(h)
                    })
        return detections


def main():
    rospy.init_node('tomato_detector', anonymous=True)
    TomatoDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

