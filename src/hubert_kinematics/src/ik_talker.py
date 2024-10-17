#!/usr/bin/env python3
# license removed for brevity
import rospy
import copy
from std_msgs.msg import UInt16
from std_msgs.msg import String
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

from hubert_kinematics.angle2pcm import Angle2pcm as _Angle2pcm
from hubert_kinematics.hubert_actions import Hubert as _Hubert

from hubert_launch.msg import LabeledPoint

from hubert_kinematics.ik_solver import ForwardKinematics as FK
from hubert_kinematics.ik_solver import InverseKinematics as IK
from math import pi
import random
import numpy as np

angle2pcm = _Angle2pcm()
hubert = _Hubert()

PI = 3.1415926536

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

def create_joint_state_msg(positions):
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['base_joint', 'neck_joint', 'cam_joint', 'shoulder_joint', 'elbow_joint']
    msg.position = positions
    msg.velocity = []
    msg.effort = []

    return msg

def joints_talker():

    sub_listener = Listener()

    pub_joint_states = rospy.Publisher('/joint_states', JointState, queue_size=10)

    pub_body = rospy.Publisher('/servo_body', UInt16, queue_size=10, latch=True)
    pub_neck_tilt = rospy.Publisher('/servo_neck_tilt', UInt16, queue_size=10, latch=True)
    pub_shoulder = rospy.Publisher('/servo_shoulder', UInt16, queue_size=10)
    pub_elbow = rospy.Publisher('/servo_elbow', UInt16, queue_size=10)
    pub_gripper = rospy.Publisher('/servo_gripper', UInt16, queue_size=10)

    rospy.init_node('joints_talker', anonymous=True)

    rate = rospy.Rate(10) # 10hz

    positions = [0, 0, 0, -PI/4, PI/4]
    msg = create_joint_state_msg(positions)
    pub_joint_states.publish(msg)

    elbow_first_value = angle2pcm.elbow(-PI/4)

    shoulder_first_value = angle2pcm.shoulder(PI/8)    
    rospy.loginfo(shoulder_first_value)
    pub_shoulder.publish(shoulder_first_value)
    time.sleep(2)
    
    shoulder_first_value = angle2pcm.shoulder(PI/4)

    rospy.loginfo(elbow_first_value)
    pub_elbow.publish(elbow_first_value)
    time.sleep(2)
    rospy.loginfo(shoulder_first_value)
    pub_shoulder.publish(shoulder_first_value)
    time.sleep(2)
    pub_gripper.publish(hubert.gripper_close())



    theta1 = random.uniform(-pi/2, pi/2)
    theta2 = random.uniform(pi/6, pi/2)
    theta3 = random.uniform(-pi/2, 0)

    time.sleep(10)

    print(f"Angles chosen for theta1, theta2 and theta3: {theta1}, {theta2}, {theta3}")

    forward_kinematics = FK(theta1, theta2, theta3)
    inverse_kinematics = IK(forward_kinematics.coords[0], forward_kinematics.coords[1], forward_kinematics.coords[2])
    angles = inverse_kinematics.angles

    positions = [angles[0], 0, 0, angles[1], angles[2]]
    msg = create_joint_state_msg(positions)

    print(f"Final angles given to Hubert for theta1, theta2 and theta3: {angles[0]}, {angles[1]}, {angles[2]}")

    pub_joint_states.publish(msg)

    rate.sleep()


if __name__ == '__main__':
    try:
        joints_talker()
    except rospy.ROSInterruptException:
        pass