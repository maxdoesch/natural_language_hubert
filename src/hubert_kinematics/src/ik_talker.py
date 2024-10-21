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
        self.sub_coords2 = rospy.Subscriber("/coordinates2", Point, self.coordinates_callback2)
        
        self.sub_instruction = rospy.Subscriber("/instruction_topic", String, self.instruction_callback)
        self.instruction = None

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

    def coordinates_callback2(self, data):

        rospy.loginfo(rospy.get_caller_id() + "I heard %s, %s, %s", data.x, data.y, data.z)
        self.coordinates_received = True
        self.coordinates = [data.x, data.y, data.z]
        
    def instruction_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I want to hear %s", data.data)
        self.instruction = data.data

def look_around(listener, pub_body, pub_neck_pan, joint_states_publisher):

    new_body_pcm = hubert.get_body_forward()
    publish(new_body_pcm, pub_body, joint_states_publisher)
    
    while listener.coordinates_received == False:
        neck_pan_pcm, body_pcm = hubert.get_cam_move_right()
        publish(body_pcm, pub_body, joint_states_publisher)
        publish(neck_pan_pcm, pub_neck_pan, joint_states_publisher)
        if neck_pan_pcm == 550:
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

    listened_instruction = "None"

    instructions = ["pick", "place"]

    print("Starting up...")

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
    
    angles = hubert.get_jointstate()
    msg = create_joint_state_msg(angles)
    pub_joint_states.publish(msg)
    print("Published joint states!")
    
    
    while not rospy.is_shutdown():

        if sub_listener.instruction == instructions[0]:
            print("pick")
            coordinates = look_around(sub_listener, pub_body, pub_neck_pan, pub_joint_states)

            [body_new_value, shoulder_new_value, elbow_new_value] = hubert.get_arm_goto([coordinates[0], coordinates[1], coordinates[2] + 0.03])

            positions = hubert.get_jointstate()
            msg = create_joint_state_msg(positions)
            pub_joint_states.publish(msg)

            publish(body_new_value, pub_body, pub_joint_states)
            publish(shoulder_new_value, pub_shoulder, pub_joint_states)
            publish(elbow_new_value, pub_elbow, pub_joint_states)

            print("Ready to grab")

            [body_new_value, shoulder_new_value, elbow_new_value] = hubert.get_arm_goto([coordinates[0], coordinates[1], coordinates[2]])

            positions = hubert.get_jointstate()
            msg = create_joint_state_msg(positions)
            pub_joint_states.publish(msg)

            publish(body_new_value, pub_body, pub_joint_states)
            publish(shoulder_new_value, pub_shoulder, pub_joint_states)
            publish(elbow_new_value, pub_elbow, pub_joint_states)

            pub_gripper.publish(hubert.get_gripper_close())

            print("Object grapped")

            [body_new_value, shoulder_new_value, elbow_new_value] = hubert.get_arm_goto([coordinates[0], coordinates[1], coordinates[2] + 0.03])

            positions = hubert.get_jointstate()
            msg = create_joint_state_msg(positions)
            pub_joint_states.publish(msg)

            publish(body_new_value, pub_body, pub_joint_states)
            publish(shoulder_new_value, pub_shoulder, pub_joint_states)
            publish(elbow_new_value, pub_elbow, pub_joint_states)

            print("Pick instruction done")
            sub_listener.instruction = "None"

        elif sub_listener.instruction == instructions[1]:
            print("place")
            # Check if place ON an object or LEFT, RIGHT
            # Check if place next to an object
            body_value = hubert.get_body_forward()  # Face straight ahead
            publish(body_value, pub_body, pub_joint_states)
            coordinates = look_around(sub_listener, pub_body, pub_neck_pan, pub_joint_states)
            # WILL ADD: Change cooridnate to a random spot around first place
            angle = random.uniform(0, 2 * np.pi)
            distance = .02  # 2 cm away
            new_x = coordinates[0] + distance * np.cos(angle)
            new_y = coordinates[1] + distance * np.sin(angle)
            coordinates = [new_x, new_y, coordinates[2]]

            [body_new_value, shoulder_new_value, elbow_new_value] = hubert.get_arm_goto(coordinates)

            publish(body_new_value, pub_body, pub_joint_states)
            publish(shoulder_new_value, pub_shoulder, pub_joint_states)
            publish(elbow_new_value, pub_elbow, pub_joint_states)

            positions = hubert.get_jointstate()
            msg = create_joint_state_msg(positions)
            pub_joint_states.publish(msg)

            print("Place instruction done")
            sub_listener.instruction = "None"



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