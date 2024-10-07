#!/usr/bin/env python3
# license removed for brevity
import rospy
import copy
from std_msgs.msg import UInt16
from std_msgs.msg import String
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

from hubert_launch.msg import LabeledPoint

# Body = 560 - 2330
# HeadPan = 550 - 2340
# HeadTilt = 950 - 2300
# Shoulder = 750 - 2200
# Elbow = 550 - 2400
# Gripper = 550 - 2150

# Right angle value for the camera: 1300

# Always take robot's movements time into consideration

# Define pi as PI
PI = 3.1415926536

def __calculate(pcm, angle_min, angle_max, pcm_min, pcm_max) -> float:
    # Raise error if PCM is outside of the boundary
    if pcm < pcm_min or pcm > pcm_max:
        raise ValueError(f"PCM must be between {pcm_min} and {pcm_max}.")

    # Linear interpolation from PCM to angle
    angle_value = angle_min + (pcm - pcm_min) * (angle_max - angle_min) / (pcm_max - pcm_min)

    return -angle_value  # Return the angle as a float

def pcm2angle_body(pcm: int) -> float:
    """
    Function that converts PCM for BODY servo to angle.
    
    :param pcm: PCM value [560, 2330]
    :return: Corresponding angle in radians.
    """

    angle_min = -PI/2
    angle_max = PI/2

    pcm_min = 560
    pcm_max = 2330

    angle_value = __calculate(pcm, angle_min, angle_max, pcm_min, pcm_max)

    return angle_value  # Return as a float

def pcm2angle_head_pan(pcm: int) -> float:
    """
    Function that converts PCM for HEAD_PAN servo to angle.
    
    :param pcm: PCM value [550, 2400]
    :return: Corresponding angle in radians.
    """

    angle_min = -PI/2
    angle_max = PI/2

    pcm_min = 550
    pcm_max = 2400

    angle_value = __calculate(pcm, angle_min, angle_max, pcm_min, pcm_max)

    return angle_value  # Return as a float

def pcm2angle_head_tilt(pcm: int) -> float:
    """
    Function that converts PCM for HEAD_TILT servo to angle.
    
    :param pcm: PCM value [950, 2400]
    :return: Corresponding angle in radians.
    """

    angle_min = -PI/4
    angle_max = PI/2

    pcm_min = 950
    pcm_max = 2400

    angle_value = __calculate(pcm, angle_min, angle_max, pcm_min, pcm_max)

    return angle_value  # Return as a float

def pcm2angle_shoulder(pcm: int) -> float:
    """
    Function that converts PCM for SHOULDER servo to angle.
    
    :param pcm: PCM value [750, 2200]
    :return: Corresponding angle in radians.
    """

    angle_min = -PI/6
    angle_max = PI/2

    pcm_min = 750
    pcm_max = 2200

    angle_value = __calculate(pcm, angle_min, angle_max, pcm_min, pcm_max)

    return -angle_value  # Return as a float

def pcm2angle_elbow(pcm: int) -> float:
    """
    Function that converts PCM for ELBOW servo to angle.
    
    :param pcm: PCM value [550, 2400]
    :return: Corresponding angle in radians.
    """

    angle_min = -PI/2
    angle_max = PI/2

    pcm_min = 550
    pcm_max = 2400

    angle_value = __calculate(pcm, angle_min, angle_max, pcm_min, pcm_max)

    return -angle_value  # Return as a float


def create_joint_state_msg(positions):
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['base_joint', 'neck_joint', 'cam_joint', 'shoulder_joint', 'elbow_joint']
    msg.position = positions
    msg.velocity = []
    msg.effort = []

    return msg

# class Label_listener:
    
#     def __init__(self):
#         self.sub = rospy.Subscriber("/label_topic", String, self.label_callback)
#         self.label_received = False

#     def label_callback(self, data):
#         rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#         self.label_received = True

# class Coordinates_listener:

#     def __init__(self, label="none"):
#         self.sub = rospy.Subscriber("/coordinates", Point, self.coordinates_callback)
#         self.coordinates_received = False
#         self.label = label

#     def coordinates_callback(self, data):

#         rospy.loginfo(rospy.get_caller_id() + "I heard %s, %s, %s", data.x, data.y, data.z)
#         self.coordinates_received = True

#     def reset(self):
#         self.sub = rospy.Subscriber("/coordinates", LabeledPoint, self.coordinates_callback)
        # self.coordinates_received = False

class Listener:
    def __init__(self):
        self.sub_label = rospy.Subscriber("/label_topic", String, self.label_callback)
        self.sub_coords = rospy.Subscriber("/coordinates", LabeledPoint, self.coordinates_callback)
        self.coordinates_received = False
        self.label_received = False
    
    def label_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.label = str(data.data)
        print(self.label)
        self.label_received = True

    def coordinates_callback(self, data):
        
        rospy.loginfo(rospy.get_caller_id() + "I heard %s, %s, %s", data.point.x, data.point.y, data.point.z)
        if data.label == self.label:
            self.coordinates_received = True
    

def joints_talker():

    pub_body = rospy.Publisher('/servo_body', UInt16, queue_size=10, latch=True)

    pub_neck_tilt = rospy.Publisher('/servo_neck_tilt', UInt16, queue_size=10, latch=True)

    pub_elbow = rospy.Publisher('/servo_elbow', UInt16, queue_size=10)

    pub_joint_states = rospy.Publisher('/joint_states', JointState, queue_size=10)

    # sub_coordinates = Coordinates_listener()

    # sub_label = Label_listener()

    sub_listener = Listener()

    rospy.init_node('body_joint_talker', anonymous=True)

    rate = rospy.Rate(10) # 10hz

    z_next_value = 1450 # Mean value would actually be 1425
    neck_tilt_value = 1300 
    elbow_value = 1400

    rospy.loginfo(z_next_value)
    pub_body.publish(z_next_value)
    rospy.loginfo(neck_tilt_value)
    pub_neck_tilt.publish(neck_tilt_value)

    positions = [pcm2angle_body(z_next_value), 0, pcm2angle_head_tilt(neck_tilt_value), 0, pcm2angle_elbow(elbow_value)]
    msg = create_joint_state_msg(positions)

    time.sleep(5)

    pub_joint_states.publish(msg)

    while not rospy.is_shutdown():
        if sub_listener.label_received == True:

            elbow_value = 1400
            rospy.loginfo(elbow_value)
            pub_elbow.publish(elbow_value)

            while sub_listener.coordinates_received == False:

                z_next_value = z_next_value + 200

                if z_next_value > 2330:
                    z_next_value = 560

                rospy.loginfo(z_next_value)
                pub_body.publish(z_next_value)

                positions = [pcm2angle_body(z_next_value), 0, pcm2angle_head_tilt(neck_tilt_value), 0, pcm2angle_elbow(elbow_value)]
                msg = create_joint_state_msg(positions)
                pub_joint_states.publish(msg)

                if z_next_value == 560:
                    time.sleep(8)
                    pub_joint_states.publish(msg)

                time.sleep(5)
                pub_joint_states.publish(msg)
                pub_joint_states.publish(msg)

            if sub_listener.coordinates_received == True:
                if elbow_value != 2200:
                    elbow_value = 2200
                rospy.loginfo(elbow_value)
                pub_elbow.publish(elbow_value)

                positions = [pcm2angle_body(z_next_value), 0, pcm2angle_head_tilt(neck_tilt_value), 0, pcm2angle_elbow(elbow_value)]
                msg = create_joint_state_msg(positions)
                pub_joint_states.publish(msg)

                time.sleep(2)

                sub_listener.label_received = False
                sub_listener.coordinates_received = False
                time.sleep(0.5)

        rate.sleep()

if __name__ == '__main__':
    try:
        joints_talker()
    except rospy.ROSInterruptException:
        pass