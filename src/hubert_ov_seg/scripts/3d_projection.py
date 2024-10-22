import numpy as np

import rospy
from sensor_msgs.msg import Image, CameraInfo
from hubert_launch.msg import LabeledPoint
from geometry_msgs.msg import Point, PointStamped, TransformStamped, PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
import image_geometry
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_matrix

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
        self.point_pub = rospy.Publisher('/base_frame/3d_coordinate', LabeledPoint, queue_size=10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.plane_base_point = None
        self.plane_normal = None
        self.cam_base_transform = None

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.camera_model.fromCameraInfo(msg)

    def get_transform(self, target_frame, source_frame, timeout=rospy.Duration(1.0)):
        try:
            # Wait for the transform to become available
            transform = self.tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time(0), timeout)
            return transform
        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform from {source_frame} to {target_frame} not found: {e}")
        except tf2_ros.ConnectivityException as e:
            rospy.logerr(f"Connectivity error looking up transform: {e}")
        except tf2_ros.ExtrapolationException as e:
            rospy.logerr(f"Extrapolation error looking up transform: {e}")
        except Exception as e:
            rospy.logerr(f"An unexpected error occurred: {e}")
        return None

    def apriltag_callback(self, msg):

        if len(msg.detections) > 0:
            point = msg.detections[0].pose.pose.pose.position

            point_camera = PointStamped()
            point_camera.header.frame_id = 'camera_link'
            point_camera.header.stamp = rospy.Time.now()
            point_camera.point = point

            transform = self.get_transform(target_frame='base_link', source_frame='camera_link')

            if transform is not None:
                self.cam_base_transform = transform
                self.plane_base_point = tf2_geometry_msgs.do_transform_point(point_camera, self.cam_base_transform)
                self.plane_base_point = np.array([self.plane_base_point.point.x, self.plane_base_point.point.y, self.plane_base_point.point.z])
                self.plane_normal = np.array([0, 0, 1])

    def pixel_coordinate_callback(self, msg):
        self.pixel_coordinate = msg

        if self.camera_info is None or self.image is None:
            rospy.logwarn("Camera info or image not received yet")
            return

        ray_cam = self.camera_model.projectPixelTo3dRay((msg.point.x, msg.point.y))

        if self.cam_base_transform is not None and self.plane_base_point is not None and self.plane_normal is not None:

            rotation = self.cam_base_transform.transform.rotation
            rotation_matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])[:3, :3]
            translation = self.cam_base_transform.transform.translation
            ray_origin = np.array([translation.x, translation.y, translation.z])

            ray_cam = np.array(ray_cam)
            ray_base = np.dot(rotation_matrix, ray_cam)
            ray_base = ray_base / np.linalg.norm(ray_base)

            denominator = np.dot(ray_base, self.plane_normal)

            t = np.dot(self.plane_base_point - ray_origin, self.plane_normal) / denominator

            intersection_point = ray_origin + t * ray_base

            point = Point(intersection_point[0], intersection_point[1], intersection_point[2])

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
        t.header.frame_id = "base_link"
        t.child_frame_id = point_3d.label
        t.transform.translation.x = point_3d.point.x
        t.transform.translation.y = point_3d.point.y
        t.transform.translation.z = point_3d.point.z
        t.transform.rotation.w = 1.0  # No rotation, only translation

        print(t)

        self.tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('pixel_to_3d_node')
    pixel_to_3d = PixelTo3D()
    rospy.spin()