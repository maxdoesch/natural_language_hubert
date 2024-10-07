#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('camera_publisher', anonymous=True)
        
        # Create a publisher object
        self.pub = rospy.Publisher('/image_topic', Image, queue_size=1)
        
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

    def start_publishing(self):
        while not rospy.is_shutdown():
            # Capture frame-by-frame
            ret, frame = self.camera.read()
            
            if ret:
                # Convert OpenCV image to ROS Image message
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                
                # Publish the image
                self.pub.publish(ros_image)
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