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
from hubert_actions import Hubert as _Hubert
from hubert_kinematics.pcm2angle import Pcm2angle as _Pcm2angle

from hubert_launch.msg import LabeledPoint

from hubert_kinematics.ik_solver import ForwardKinematics as FK
from hubert_kinematics.ik_solver import InverseKinematics as IK
from math import pi
import random
import numpy as np

angle2pcm = _Angle2pcm()
pcm2angle = _Pcm2angle()
hubert = _Hubert()

PI = 3.1415926536

class Listener:
    def __init__(self):
        self.sub_label = rospy.Subscriber("/label_topic", String, self.label_callback)
        self.sub_coords = rospy.Subscriber("/coordinates", LabeledPoint, self.coordinates_callback)
        self.coordinates_received = False
        self.label_received = False
        self.coordinates = [None, None, None]
    
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
            self.coordinates = [data.point.x, data.point.y, data.point.z]
        else:
            self.coordinates_received = False


def look_around(listener, pub_neck_pan, joint_states_publisher):
    
    while listener.coordinates_received == False:
        neck_pan_value = hubert.neck_pan_move_right()
        publish(neck_pan_value, pub_neck_pan, joint_states_publisher)
        if neck_pan_value == 550:
            time.sleep(5)
    
    if listener.coordinates_received == True:
        return listener.coordinates


def publish(pcm_value, publisher, joint_states_publisher, delay=2):
    publisher.publish(pcm_value)
    joint_states_publisher.publish(create_joint_state_msg(hubert.get_jointstate()))
    time.sleep(delay)

def create_joint_state_msg(positions):
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['base_joint', 'cam_joint', 'neck_joint', 'shoulder_joint', 'elbow_joint']
    msg.position = positions
    msg.velocity = []
    msg.effort = []

    return msg

def joints_talker():

    sub_listener = Listener()

    pub_joint_states = rospy.Publisher('/joint_states', JointState, queue_size=10)

    pub_body = rospy.Publisher('/servo_body', UInt16, queue_size=10, latch=True)
    pub_neck_tilt = rospy.Publisher('/servo_neck_tilt', UInt16, queue_size=10, latch=True)
    pub_neck_pan = rospy.Publisher('/servo_neck_pan', UInt16, queue_size=10, latch=True)
    pub_shoulder = rospy.Publisher('/servo_shoulder', UInt16, queue_size=10)
    pub_elbow = rospy.Publisher('/servo_elbow', UInt16, queue_size=10)
    pub_gripper = rospy.Publisher('/servo_gripper', UInt16, queue_size=10)

    rospy.init_node('joints_talker', anonymous=True)

    rate = rospy.Rate(10) # 10hz

    instructions = ["pick", "place"]

    # positions = [0, 0, 0, -PI/4, PI/4]
    # msg = create_joint_state_msg(positions)
    # pub_joint_states.publish(msg)

    # elbow_first_value = angle2pcm.elbow(-PI/4)

    # shoulder_first_value = angle2pcm.shoulder(PI/8)    
    # rospy.loginfo(shoulder_first_value)
    # pub_shoulder.publish(shoulder_first_value)
    # time.sleep(2)

    # shoulder_first_value = angle2pcm.shoulder(PI/4)

    # rospy.loginfo(elbow_first_value)
    # pub_elbow.publish(elbow_first_value)
    # time.sleep(2)
    # rospy.loginfo(shoulder_first_value)
    # pub_shoulder.publish(shoulder_first_value)
    # time.sleep(2)
    # pub_gripper.publish(hubert.gripper_close())

    [body_value, neck_tilt_value, neck_pan_value, shoulder_value, elbow_value, gripper_value] = hubert.get_stance_first()
    
    pub_body.publish(body_value)
    time.sleep(1)
    pub_neck_tilt.publish(neck_tilt_value)
    time.sleep(1)
    pub_neck_pan.publish(neck_pan_value)
    time.sleep(1)
    pub_shoulder.publish(shoulder_value)
    time.sleep(1)
    pub_elbow.publish(elbow_value)
    time.sleep(1)
    pub_gripper.publish(gripper_value)

    #Publish on joint states

    time.sleep(4)
    print("First stance done!")

    neck_tilt_value = hubert.get_neck_tilt_down()
    pub_neck_tilt.publish(neck_tilt_value)
    time.sleep(4)
    print("Tilt neck done!")

    [shoulder_value, elbow_value] = hubert.get_arm_idle()
    pub_elbow.publish(elbow_value)
    time.sleep(1)
    pub_shoulder.publish(shoulder_value)
    
    print("Idle arms done!")
    time.sleep(4)

    angles = [pcm2angle.body(body_value), pcm2angle.neck_tilt(neck_tilt_value), pcm2angle.neck_pan(neck_pan_value),
                       pcm2angle.shoulder(shoulder_value), pcm2angle.elbow(elbow_value)]
    

    while not rospy.is_shutdown():

        if listened_instruction == instructions[0]:
            print("pick")
            coordinates = look_around(sub_listener, pub_neck_pan, pub_joint_states)
            inverse_kinematics = IK(coordinates[0], coordinates[1], coordinates[2] + 0.03)
            angles = inverse_kinematics.angles

            body_new_value = angle2pcm.body(angles[0])
            shoulder_new_value = angle2pcm.shoulder(angles[1])
            elbow_new_value = angle2pcm.elbow(angles[2])

            positions = [angles[0], 0, 0, angles[1], angles[2]]
            msg = create_joint_state_msg(positions)
            pub_joint_states.publish(msg)

            publish(body_new_value, pub_body, pub_joint_states)
            publish(shoulder_new_value, pub_shoulder, pub_joint_states)
            publish(elbow_new_value, pub_elbow, pub_joint_states)

            inverse_kinematics = IK(coordinates[0], coordinates[1], coordinates[2])
            angles = inverse_kinematics.angles

            body_new_value = angle2pcm.body(angles[0])
            shoulder_new_value = angle2pcm.shoulder(angles[1])
            elbow_new_value = angle2pcm.elbow(angles[2])

            positions = [angles[0], 0, 0, angles[1], angles[2]]
            msg = create_joint_state_msg(positions)
            pub_joint_states.publish(msg)

            publish(body_new_value, pub_body, pub_joint_states)
            publish(shoulder_new_value, pub_shoulder, pub_joint_states)
            publish(elbow_new_value, pub_elbow, pub_joint_states)

            pub_gripper.publish(_Hubert.gripper_close())


    # while True:
    #     msg = create_joint_state_msg(angles)
    #     pub_joint_states.publish(msg)

    theta1 = random.uniform(-pi/4, pi/2)
    theta2 = random.uniform(pi/4, pi/2)
    theta3 = random.uniform(-pi/4, 0)

    time.sleep(5)

    print(f"Angles chosen for theta1, theta2 and theta3: {theta1}, {theta2}, {theta3}")

    forward_kinematics = FK(0.64683544, 1.15652211, -1.07848157)
    inverse_kinematics = IK(forward_kinematics.coords[0], forward_kinematics.coords[1], forward_kinematics.coords[2] + 0.03)
    angles = inverse_kinematics.angles
    
    # body_value, shoulder_value, elbow_value = hubert.move_arm(coordinates)
    # publish(body_value)


    time.sleep(5)

    positions = [angles[0], 0, 0, angles[1], angles[2]]
    above_positions = positions.copy()
    above_angles = angles.copy()
    msg = create_joint_state_msg(positions)

    body_new_value = angle2pcm.body(angles[0])
    shoulder_new_value = angle2pcm.shoulder(angles[1])
    elbow_new_value = angle2pcm.elbow(angles[2])

    rospy.loginfo(body_new_value)
    pub_body.publish(body_new_value)
    time.sleep(1)
    rospy.loginfo(shoulder_new_value)
    pub_shoulder.publish(shoulder_new_value)
    time.sleep(1)
    rospy.loginfo(elbow_new_value)
    pub_elbow.publish(elbow_new_value)

    pub_joint_states.publish(msg)

    print(f"Intermediate angles given to Hubert for theta1, theta2 and theta3: {angles[0]}, {angles[1]}, {angles[2]}")

    time.sleep(2)
    inverse_kinematics = IK(forward_kinematics.coords[0], forward_kinematics.coords[1], forward_kinematics.coords[2])
    angles = inverse_kinematics.angles
    
    positions = [angles[0], 0, 0, angles[1], angles[2]]
    msg = create_joint_state_msg(positions)

    body_new_value = angle2pcm.body(angles[0])
    shoulder_new_value = angle2pcm.shoulder(angles[1])
    elbow_new_value = angle2pcm.elbow(angles[2])

    publish(body_new_value, pub_body)

    rospy.loginfo(elbow_new_value)
    pub_elbow.publish(elbow_new_value)
    time.sleep(1)
    rospy.loginfo(shoulder_new_value)
    pub_shoulder.publish(shoulder_new_value)
    time.sleep(1)

    pub_gripper.publish(_Hubert.gripper_close())

    pub_joint_states.publish(msg)
    time.sleep(5)

    positions = above_positions
    angles = above_angles

    msg = create_joint_state_msg(positions)

    body_new_value = angle2pcm.body(angles[0])
    shoulder_new_value = angle2pcm.shoulder(angles[1])
    elbow_new_value = angle2pcm.elbow(angles[2])

    rospy.loginfo(shoulder_new_value)
    pub_shoulder.publish(shoulder_new_value)
    time.sleep(1)
    rospy.loginfo(elbow_new_value)
    pub_elbow.publish(elbow_new_value)
    time.sleep(1)
    rospy.loginfo(body_new_value)
    pub_body.publish(body_new_value)
    time.sleep(1)

    [body_value, neck_tilt_value, neck_pan_value, shoulder_value, elbow_value, gripper_value] = hubert.stance_first()
    
    pub_body.publish(body_value)
    time.sleep(1)
    pub_neck_tilt.publish(neck_tilt_value)
    time.sleep(1)
    pub_neck_pan.publish(neck_pan_value)
    time.sleep(1)
    pub_shoulder.publish(shoulder_value)
    time.sleep(1)
    pub_elbow.publish(elbow_value)
    time.sleep(1)
    pub_gripper.publish(gripper_value)
    time.sleep(4)
    print("First stance done!")
    

    

    

    
    print(f"Final angles given to Hubert for theta1, theta2 and theta3: {angles[0]}, {angles[1]}, {angles[2]}")    

    rate.sleep()


if __name__ == '__main__':
    try:
        joints_talker()
    except rospy.ROSInterruptException:
        pass