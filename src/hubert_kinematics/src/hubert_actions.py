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
    def __init__(self, joint_publisher_list, joint_states_publisher):
        """
        Hubert stance class
        
        Hubert.stance_...: Methods that returns whole stances
        Hubert.body_...: Methods that returns body movement
        Hubert.arm_...: Methods that returns arm movements (both shoulder and elbow)
        Hubert.neck_...: Methods that returns head movements (both tilt and pan)

        Parameters:
        joint_publisher_list : List of every publisher as [body, neck_tilt, neck_pan, shoulder, elbow, gripper]
        joint_states_publisher : The joint states publisher
        
        """



        pass

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

