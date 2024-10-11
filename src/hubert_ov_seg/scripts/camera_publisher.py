#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import cv2

class CameraPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('camera_publisher', anonymous=True)

        # Create a publisher object
        self.pub_image_raw = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        self.pub_camera_info = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=1 ,latch=True)
        
        # Create a CvBridge object
        self.bridge = CvBridge()
        
        # Open the camera
        self.camera = cv2.VideoCapture(0)  # 0 for default camera
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        if not self.camera.isOpened():
            rospy.logerr("Could not open camera")
            rospy.signal_shutdown("Camera not opened")
        
        # Set the publishing rate
        self.rate = rospy.Rate(30)  # 30 Hz

    def create_camera_info(self):
        # Initialize the CameraInfo message
        camera_info = CameraInfo()

        # Set image dimensions
        camera_info.width = 640
        camera_info.height = 480

        # Distortion coefficients: D
        # [k1, k2, p1, p2, k3]
        camera_info.distortion_model = "plumb_bob"
        camera_info.D = [
            -0.023199777932122682,  # k1
            -0.025918697269801137,  # k2
            0.001767049052526465,   # p1
            -0.0012548114794264373, # p2
            0.0                      # k3
        ]

        # Intrinsic camera matrix: K
        # [fx,  0, cx,
        #   0, fy, cy,
        #   0,  0,  1]
        camera_info.K = [
            672.8186259811123, 0.0, 313.24906274715994,
            0.0, 672.4172018457331, 256.6239831156741,
            0.0, 0.0, 1.0
        ]

        # Rectification matrix: R
        # Identity matrix since it's a mono camera
        camera_info.R = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # Projection matrix: P
        # [fx',  0, cx', Tx,
        #   0, fy', cy', Ty,
        #   0,   0,   1,  0]
        camera_info.P = [
            668.3340553377487, 0.0, 312.55442571781094, 0.0,
            0.0, 669.8883102878623, 257.2140328011053, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        # Optional: Set the binning and ROI if needed
        camera_info.binning_x = 0
        camera_info.binning_y = 0
        camera_info.roi.x_offset = 0
        camera_info.roi.y_offset = 0
        camera_info.roi.height = 0
        camera_info.roi.width = 0
        camera_info.roi.do_rectify = False

        # Set the frame ID and timestamp if necessary
        camera_info.header.frame_id = "camera_frame"
        camera_info.header.stamp = rospy.Time.now()

        return camera_info

    def start_publishing(self):
        while not rospy.is_shutdown():
            # Capture frame-by-frame
            ret, frame = self.camera.read()
            
            if ret:
                # Convert OpenCV image to ROS Image message
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                
                # Publish the image
                self.pub_image_raw.publish(ros_image)
                self.pub_camera_info.publish(self.create_camera_info())
            else:
                rospy.logerr("Failed to capture image")
            
            # Sleep to maintain the desired rate
            self.rate.sleep()

    def __del__(self):
        # Release the camera when the object is deleted
        self.camera.release()

if __name__ == '__main__':
    try:
        camera_publisher = CameraPublisher()
        camera_publisher.start_publishing()
    except rospy.ROSInterruptException:
        pass