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

from hubert_kinematics.ik_solver import ForwardKinematics as FK
from hubert_kinematics.ik_solver import InverseKinematics as IK
from math import pi
import random
import numpy as np

def create_joint_state_msg(positions):
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['base_joint', 'neck_joint', 'cam_joint', 'shoulder_joint', 'elbow_joint']
    msg.position = positions
    msg.velocity = []
    msg.effort = []

    return msg

def joints_talker():

    pub_joint_states = rospy.Publisher('/joint_states', JointState, queue_size=10)

    rospy.init_node('rviz_joints_talker', anonymous=True)

    rate = rospy.Rate(10) # 10hz

    theta1 = random.uniform(-pi/2, pi/2)
    theta2 = random.uniform(pi/6, pi/2)
    theta3 = random.uniform(-pi/2, 0)

    print(f"Angles chosen for theta1, theta2 and theta3: {theta1}, {theta2}, {theta3}")

    forward_kinematics = FK(theta1, theta2, theta3)
    inverse_kinematics = IK(forward_kinematics.coords[0], forward_kinematics.coords[1], forward_kinematics.coords[2])
    angles = inverse_kinematics.angles

    positions = [angles[0], 0, 0, angles[1], angles[2]]
    msg = create_joint_state_msg(positions)

    print(f"Final angles given to Hubert for theta1, theta2 and theta3: {angles[0]}, {angles[1]}, {angles[2]}")

    pub_joint_states.publish(msg)


if __name__ == '__main__':
    try:
        joints_talker()
    except rospy.ROSInterruptException:
        pass