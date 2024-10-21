#!/usr/bin/env python3
# license removed for brevity
import rospy
import copy
from std_msgs.msg import UInt16
from std_msgs.msg import String
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

from hubert_kinematics import pcm2angle

from hubert_launch.msg import LabeledPoint

# Body = 560 - 2330
# HeadPan = 550 - 2340
# HeadTilt = 950 - 2300
# Shoulder = 750 - 2200
# Elbow = 550 - 2400
# Gripper = 550 - 2150

# Right angle value for the camera: 1300

# Always take robot's movements time into consideration

def create_joint_state_msg(positions):
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['base_joint', 'neck_joint', 'cam_joint', 'shoulder_joint', 'elbow_joint']
    msg.position = positions
    msg.velocity = []
    msg.effort = []

    return msg

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
        self.coordinates_received = False

    def coordinates_callback(self, data):
        
        rospy.loginfo(rospy.get_caller_id() + "I heard %s, %s, %s, %s", data.point.x, data.point.y, data.point.z, data.label)
        rospy.loginfo(rospy.get_caller_id() + "I want to hear %s", self.label)
        if data.label == self.label:
            self.coordinates_received = True
            print('fjdskljfkls')
        else:
            self.coordinates_received = False

    

def joints_talker():

    pub_body = rospy.Publisher('/servo_body', UInt16, queue_size=10, latch=True)

    pub_neck_tilt = rospy.Publisher('/servo_neck_tilt', UInt16, queue_size=10, latch=True)

    pub_elbow = rospy.Publisher('/servo_elbow', UInt16, queue_size=10)

    pub_joint_states = rospy.Publisher('/joint_states', JointState, queue_size=10)

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

    positions = [pcm2angle.body(z_next_value), 0, pcm2angle.head_tilt(neck_tilt_value), 0, pcm2angle.elbow(elbow_value)]
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

                positions = [pcm2angle.body(z_next_value), 0, pcm2angle.head_tilt(neck_tilt_value), 0, pcm2angle.elbow(elbow_value)]
                msg = create_joint_state_msg(positions)
                pub_joint_states.publish(msg)

                if z_next_value == 560:
                    time.sleep(8)
                    pub_joint_states.publish(msg)

                time.sleep(5)
                pub_joint_states.publish(msg)
                pub_joint_states.publish(msg)

            if sub_listener.coordinates_received == True:

                sub_listener.label_received = False
                sub_listener.coordinates_received = False

                if elbow_value != 2200:
                    elbow_value = 2200
                rospy.loginfo(elbow_value)
                pub_elbow.publish(elbow_value)

                positions = [pcm2angle.body(z_next_value), 0, pcm2angle.head_tilt(neck_tilt_value), 0, pcm2angle.elbow(elbow_value)]
                msg = create_joint_state_msg(positions)
                pub_joint_states.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        joints_talker()
    except rospy.ROSInterruptException:
        pass