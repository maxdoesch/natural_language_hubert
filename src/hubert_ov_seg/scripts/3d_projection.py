import numpy as np

import rospy
from sensor_msgs.msg import Image, CameraInfo
from hubert_launch.msg import LabeledPoint
from geometry_msgs.msg import Point, TransformStamped
from apriltag_ros.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
import image_geometry
import tf2_ros

class PixelTo3D:
    def __init__(self):
        self.bridge = CvBridge()
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_info = None
        self.image = None
        self.pixel_coordinate = None

        # Subscribers
        rospy.Subscriber('/hubert_camera/image_rect_color', Image, self.image_callback)
        rospy.Subscriber('/hubert_camera/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/hubert_camera/pixel_coordinate', LabeledPoint, self.pixel_coordinate_callback)
        rospy.Subscriber('/hubert_camera/tag_detections', AprilTagDetectionArray, self.apriltag_callback), 

        # Publisher
        self.point_pub = rospy.Publisher('/hubert_camera/3d_coordinate', LabeledPoint, queue_size=10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.distance = None

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.camera_model.fromCameraInfo(msg)

    def apriltag_callback(self, msg):

        if len(msg.detections) > 0:
            point = msg.detections[0].pose.pose.pose.position

            self.distance = np.sqrt(point.x**2 + point.y**2 + point.z**2)

    def pixel_coordinate_callback(self, msg):
        self.pixel_coordinate = msg

        if self.camera_info is None or self.image is None:
            rospy.logwarn("Camera info or image not received yet")
            return

        # Example pixel coordinate (modify as needed)
        pixel_x, pixel_y = msg.point.x, msg.point.y

        # Project pixel to 3D ray
        ray = self.camera_model.projectPixelTo3dRay((pixel_x, pixel_y))

        distance = self.distance if self.distance is not None else 1

        # Scale the ray by the distance
        point = Point(ray[0] * distance, ray[1] * distance, ray[2] * distance)
        point_3d = LabeledPoint()
        point_3d.label = msg.label
        point_3d.point = point

        # Publish the 3D point
        self.point_pub.publish(point_3d)

        # Publish TF frame
        self.publish_tf_frame(point_3d)

        rospy.loginfo(f"Published 3D point: {point_3d}")

    def publish_tf_frame(self, point_3d):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "cam_link"
        t.child_frame_id = point_3d.label
        t.transform.translation.x = point_3d.point.x
        t.transform.translation.y = point_3d.point.y
        t.transform.translation.z = point_3d.point.z
        t.transform.rotation.w = 1.0  # No rotation, only translation

        self.tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('pixel_to_3d_node')
    pixel_to_3d = PixelTo3D()
    rospy.spin()