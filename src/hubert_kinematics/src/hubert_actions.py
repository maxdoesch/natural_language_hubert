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
from hubert_kinematics.pcm2angle import Pcm2angle as _Pcm2angle
import numpy as np
from math import pi

from hubert_kinematics.ik_solver import InverseKinematics as IK

angle2pcm = _Angle2pcm()
pcm2angle = _Pcm2angle()

class Hubert:
    def __init__(self, joint_publisher_list, jointstate_publisher):
        """
        Hubert stance class

        Parameters
        ----------
        joint_publisher_list : list
            List of every publisher as [body, neck_tilt, neck_pan, shoulder, elbow, gripper]
        jointstate_publisher : rospy.Publisher
            The joint states publisher
        """

        # Publishers
        self.pub_body = joint_publisher_list[0]
        self.pub_neck_tilt = joint_publisher_list[1]
        self.pub_neck_pan = joint_publisher_list[2]
        self.pub_shoulder = joint_publisher_list[3]
        self.pub_elbow = joint_publisher_list[4]
        self.pub_gripper = joint_publisher_list[5]
        self.pub_jointstate = jointstate_publisher

        # PCM values for each joint
        self.pcm_body = 0
        self.pcm_neck_tilt = 0
        self.pcm_neck_pan = 0
        self.pcm_shoulder = 0
        self.pcm_elbow = 0
        self.pcm_gripper = 0
        
        # Angle values for each joint
        self.angle_body = 0
        self.angle_neck_tilt = 0
        self.angle_neck_pan = 0
        self.angle_shoulder = 0
        self.angle_elbow = 0
        self.angle_gripper = 0
    
    def update_positions(self):
        """Updates the positions of the Hubert robot and publishes the joint states"""
        positions_angle = [self.angle_body, self.angle_neck_tilt, self.angle_neck_pan, self.angle_shoulder, self.angle_elbow]
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ['base_joint', 'cam_joint', 'neck_joint', 'shoulder_joint', 'elbow_joint']
        msg.position = positions_angle
        msg.velocity = []
        msg.effort = []

        self.pub_jointstate.publish(msg)

    def move_arm(self, coordinates):

        inverse_kinematics = IK(coordinates[0], coordinates[1], coordinates[2] + 0.03)
        angles = inverse_kinematics.angles

        time.sleep(5)

        positions = [angles[0], 0, 0, angles[1], angles[2]]
        above_positions = positions.copy()
        above_angles = angles.copy()
        msg = self.create_joint_state_msg(positions)

        body_new_value = angle2pcm.body(angles[0])
        shoulder_new_value = angle2pcm.shoulder(angles[1])
        elbow_new_value = angle2pcm.elbow(angles[2])

        rospy.loginfo(body_new_value)
        self.pub_body.publish(body_new_value)
        time.sleep(1)
        rospy.loginfo(shoulder_new_value)
        self.pub_shoulder.publish(shoulder_new_value)
        time.sleep(1)
        rospy.loginfo(elbow_new_value)
        self.pub_elbow.publish(elbow_new_value)

        self.pub_joint_states.publish(msg)


    @staticmethod
    def gripper_open():
        """Returns the pcm value that corresponds to a fully opened gripper"""
        return angle2pcm.gripper(33)
    
    @staticmethod
    def gripper_close():
        """Returns the pcm value that corresponds to a fully closed gripper"""
        return angle2pcm.gripper(0)
    
    @staticmethod
    def arm_idle():
        """Returns the pcm values for [shoulder, elbow] for idle position in arms"""
        return np.array([angle2pcm.shoulder(pi/2), angle2pcm.elbow(-pi/2)])
    
    def arm_lift():
        """DO NOT USE"""
        pass
    
    @staticmethod
    def neck_tilt_down():
        """Returns the pcm value that corresponds to the angle of looking down towards the workspace"""
        return 1300 # Value is hardcoded
    
    @staticmethod
    def stance_first(return_angles=False):
        """Returns a fully idle stance with 0 angles on all servos
        
        Returns [body, neck_tilt, neck_pan, shoulder, elbow, gripper]

        """
        return np.array([
            angle2pcm.body(),
            angle2pcm.neck_tilt(),
            angle2pcm.neck_pan(),
            angle2pcm.shoulder(),
            angle2pcm.elbow(),
            angle2pcm.gripper(33)
        ])

