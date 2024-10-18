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
    
    def get_jointstate(self):
        """Updates the positions of the Hubert robot and publishes the joint states"""
        positions_angle = [self.angle_body, self.angle_neck_tilt, self.angle_neck_pan, self.angle_shoulder, self.angle_elbow]
        return positions_angle

    def move_arm(self, coordinates):

        inverse_kinematics = IK(coordinates[0], coordinates[1], coordinates[2] + 0.03)

        [self.angle_body, self.angle_shoulder, self.angle_elbow] = inverse_kinematics.angles

        self.pcm_body = angle2pcm.body(self.angle_body)
        self.pcm_shoulder = angle2pcm.shoulder(self.angle_body)
        self.pcm_elbow = angle2pcm.elbow(self.angle_elbow)

        time.sleep(5)
        above_positions = positions.copy()
        above_angles = angles.copy()
        self.update_positions()

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

