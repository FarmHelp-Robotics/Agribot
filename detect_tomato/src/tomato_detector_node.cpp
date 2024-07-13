#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vision_msgs/BoundingBox2D.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

class TomatoDetector {
public:
    TomatoDetector() {
        ros::NodeHandle nh("~");
        std::string image_topic;
        nh.param("image_topic", image_topic, std::string("/camera/rgb/image_raw"));
        image_sub_ = nh.subscribe(image_topic, 1, &TomatoDetector::imageCallback, this);
        detection_pub_ = nh.advertise<vision_msgs::BoundingBox2D>("/tomato_detection", 10);

        // Load your YOLO model here
        net_ = cv::dnn::readNetFromONNX("path/to/your/model.onnx");
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;

        // Preprocess the frame and run the detection
        cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true, false);
        net_.setInput(blob);
        std::vector<cv::Mat> detections;
        net_.forward(detections, getOutputsNames(net_));

        // Process detections
        for (size_t i = 0; i < detections.size(); ++i) {
            float* data = (float*)detections[i].data;
            for (int j = 0; j < detections[i].rows; ++j, data += detections[i].cols) {
                cv::Mat scores = detections[i].row(j).colRange(5, detections[i].cols);
                cv::Point classIdPoint;
                double confidence;
                minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                if (confidence > 0.5) {
                    int centerX = (int)(data[0] * frame.cols);
                    int centerY = (int)(data[1] * frame.rows);
                    int width = (int)(data[2] * frame.cols);
                    int height = (int)(data[3] * frame.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;

                    // Create and publish the bounding box message
                    vision_msgs::BoundingBox2D bounding_box_msg;
                    bounding_box_msg.center.x = centerX;
                    bounding_box_msg.center.y = centerY;
                    bounding_box_msg.size_x = width;
                    bounding_box_msg.size_y = height;
                    detection_pub_.publish(bounding_box_msg);

                    // Optionally, display the image for debugging
                    cv::rectangle(frame, cv::Point(left, top), cv::Point(left + width, top + height), cv::Scalar(0, 255, 0), 2);
                    cv::circle(frame, cv::Point(centerX, centerY), 5, cv::Scalar(255, 0, 0), -1);
                }
            }
        }

        cv::imshow("Tomato Detection", frame);
        cv::waitKey(1);
    }

private:
    ros::Subscriber image_sub_;
    ros::Publisher detection_pub_;
    cv::dnn::Net net_;

    std::vector<std::string> getOutputsNames(const cv::dnn::Net& net) {
        static std::vector<std::string> names;
        if (names.empty()) {
            std::vector<int> outLayers = net.getUnconnectedOutLayers();
            std::vector<std::string> layersNames = net.getLayerNames();
            names.resize(outLayers.size());
            for (size_t i = 0; i < outLayers.size(); ++i)
                names[i] = layersNames[outLayers[i] - 1];
        }
        return names;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tomato_detector");
    TomatoDetector td;
    ros::spin();
    return 0;
}

