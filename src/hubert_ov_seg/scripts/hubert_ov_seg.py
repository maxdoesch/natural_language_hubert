import os
import copy
import rospy
import torch
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from hubert_launch.msg import LabeledPoint
from geometry_msgs.msg import Point

from detectron2.config import get_cfg

current_directory = os.getcwd()
new_directory = '/home/max/hubert_ws/src/hubert_ov_seg/FrozenSeg'
os.chdir(new_directory)


from demo.predictor import SlimFrozenSeg
from frozenseg import add_frozenseg_config, add_maskformer2_config

def setup_cfg():
    # load config from file and command-line arguments
    cfg = get_cfg()
    add_maskformer2_config(cfg)
    add_frozenseg_config(cfg)
    cfg.merge_from_file('/home/max/hubert_ws/src/hubert_ov_seg/FrozenSeg/configs/coco/frozenseg/convnext_large_eval_ade20k.yaml')
    opts = ["MODEL.WEIGHTS", "/home/max/hubert_ws/src/hubert_ov_seg/FrozenSeg/pretrained_checkpoint/frozenseg_ConvNeXt-Large.pth"]
    cfg.merge_from_list(opts)
    cfg.freeze()
    return cfg

def get_instance_com(id, panoptic_seg):
    mask = panoptic_seg == id
    y_indices, x_indices = torch.nonzero(mask, as_tuple=True)
    y_indices = y_indices.float()
    x_indices = x_indices.float()
    
    center_y = y_indices.mean().item()
    center_x = x_indices.mean().item()

    return (center_x, center_y)


class HuberOVSeg:
    def __init__(self):
        rospy.init_node("huber_ov_seg")
        self.cv2_bridge = CvBridge()
        cfg = setup_cfg()
        self.ov_seg_model = SlimFrozenSeg(cfg)
        self.mask_pub = rospy.Publisher('/hubert_camera/panoptic_mask', Image, queue_size=1)
        self.coordinate_publisher = rospy.Publisher('/hubert_camera/pixel_coordinate', LabeledPoint, queue_size=1)
        self.label_sub = rospy.Subscriber('/hubert/label_topic', String, self.label_callback, queue_size=1)
        self.image_sub = rospy.Subscriber('/hubert_camera/image_raw', Image, self.image_callback, queue_size=1)
        self.latest_image = None
        self.processing = False
        self.latest_label = 'Person'
        
        self.process_rate = rospy.Rate(10)
        rospy.Timer(rospy.Duration(1.0/10), self.process_image)

    def image_callback(self, msg):
        self.latest_image = msg

    def label_callback(self, msg):

        self.latest_label = msg.data

        print(self.latest_label)

    def process_image(self, event):
        if self.latest_image is not None and not self.processing:
            self.ov_seg_model.set_label(self.latest_label)

            self.processing = True
            cv2_image = self.cv2_bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
            
            predictions, visualized_output = self.ov_seg_model.run_on_image(cv2_image)

            panoptic_seg, segments_info = predictions["panoptic_seg"]

            for segment in segments_info:
                if segment['category_id'] == 0:
                    com_coordinates = get_instance_com(segment['id'], panoptic_seg)

                    labeled_point = LabeledPoint()
                    labeled_point.point = Point(x=com_coordinates[0], y=com_coordinates[1])
                    labeled_point.label = self.latest_label

                    self.coordinate_publisher.publish(labeled_point)

                    continue


            ros_image = self.cv2_bridge.cv2_to_imgmsg(visualized_output.get_image()[:, :, ::-1], encoding="bgr8")
            ros_image.header.frame_id = "camera_link"
            self.mask_pub.publish(ros_image)
            self.processing = False
            self.latest_image = None

if __name__ == '__main__':
    try:
        huber_ov_seg = HuberOVSeg()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass