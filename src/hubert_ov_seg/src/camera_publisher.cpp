#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>

#define CAMERA_FRAME "camera_link"

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
        camera_info.D = {-0.26356547589441004, 0.38262175472105336, -0.0028605597622201632, -0.0014097888518859031, 0.0};
        camera_info.K = {1036.662690584779, 0.0, 287.6858475723572,
                        0.0, 1032.3445046229688, 222.55446891878728,
                        0.0, 0.0, 1.0};
        camera_info.R = {1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0};
        camera_info.P = {1012.5476517938175, 0.0, 286.00028020439396, 0.0,
                        0.0, 1017.6788592402461, 221.62335510675115, 0.0,
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