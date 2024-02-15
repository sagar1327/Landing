#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>

class ImageSubscriber {
public:
    ImageSubscriber() {
        nh_ = ros::NodeHandle("~");
        image_sub_ = nh_.subscribe("/iris_downward_depth_camera/camera/rgb/image_raw/compressed", 1000, &ImageSubscriber::imageCallback, this);

        image_pub_ = nh_.advertise<sensor_msgs::Image>("/fractal_marker/rgb/image_raw", 1);
    }

    void imageCallback(const sensor_msgs::CompressedImageConstPtr& compressed_image) {
        try {
            cv::Mat image = cv::imdecode(cv::Mat(compressed_image->data), 1000);

            aruco::FractalDetector FDetector;
            FDetector.setConfiguration("/home/sagar/ROS/src/offboard_py/src/fractal_marker_3L.yml");
            if(FDetector.detect(image)){
                FDetector.drawMarkers(image);
            }

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            image_pub_.publish(msg);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("CV Bridge Exception: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_subscriber_node");
    ImageSubscriber image_subscriber;
    ros::spin();

    return 0;
}