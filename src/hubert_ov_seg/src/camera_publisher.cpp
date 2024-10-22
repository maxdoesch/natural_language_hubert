#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>

#define CAMERA_FRAME "hubert_camera"

class CameraPublisher {
public:
    CameraPublisher() : nh_("~"), rate_(30) {
        image_transport::ImageTransport it(nh_);
        pub_image = it.advertiseCamera("/hubert_camera/image_raw", 1, true);

        camera_.open(0);  // Open default camera
        camera_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        if (!camera_.isOpened()) {
            ROS_ERROR("Could not open camera");
            ros::shutdown();
        }
    }

    ~CameraPublisher() {
        camera_.release();
    }

    sensor_msgs::CameraInfo createCameraInfo() {
        sensor_msgs::CameraInfo camera_info;

        camera_info.width = 640;
        camera_info.height = 480;

        camera_info.distortion_model = "plumb_bob";
        camera_info.D = {-0.023199777932122682, -0.025918697269801137, 0.001767049052526465, -0.0012548114794264373, 0.0};

        camera_info.K = {672.8186259811123, 0.0, 313.24906274715994,
                         0.0, 672.4172018457331, 256.6239831156741,
                         0.0, 0.0, 1.0};

        camera_info.R = {1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0};

        camera_info.P = {668.3340553377487, 0.0, 312.55442571781094, 0.0,
                         0.0, 669.8883102878623, 257.2140328011053, 0.0,
                         0.0, 0.0, 1.0, 0.0};

        camera_info.binning_x = 0;
        camera_info.binning_y = 0;
        camera_info.roi.x_offset = 0;
        camera_info.roi.y_offset = 0;
        camera_info.roi.height = 0;
        camera_info.roi.width = 0;
        camera_info.roi.do_rectify = false;

        return camera_info;
    }

    void startPublishing() {
        while (ros::ok()) {
            cv::Mat frame;
            if (camera_.read(frame)) {
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
                msg->header.frame_id = CAMERA_FRAME;
                msg->header.stamp = ros::Time::now();
                sensor_msgs::CameraInfoPtr info_msg(new sensor_msgs::CameraInfo(createCameraInfo()));
                info_msg->header = msg->header;
                pub_image.publish(msg, info_msg);
            } else {
                ROS_ERROR("Failed to capture image");
            }

            rate_.sleep();
            ros::spinOnce();
        }
    }

private:
    ros::NodeHandle nh_;
    image_transport::CameraPublisher pub_image;
    cv::VideoCapture camera_;
    ros::Rate rate_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_publisher");
    CameraPublisher camera_publisher;
    camera_publisher.startPublishing();
    return 0;
}