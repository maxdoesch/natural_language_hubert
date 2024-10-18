from hubert_kinematics.angle2pcm import Angle2pcm as _Angle2pcm
from hubert_kinematics.pcm2angle import Pcm2angle as _Pcm2angle
import numpy as np
from math import pi

from hubert_kinematics.ik_solver import InverseKinematics as IK

angle2pcm = _Angle2pcm()
pcm2angle = _Pcm2angle()

class Hubert:
    def _init_(self):
        """
        Hubert stance class

        Gives pcm values (or angles) for given tasks. 
        Methods:
        --------
        stance - Returns every joint values for a stance
        arm - Returns three joints (body, shoulder, elbow) for a stance

        """

        # PCM values for each joint
        self._pcm_body = 0
        self._pcm_neck_tilt = 0
        self._pcm_neck_pan = 0
        self._pcm_shoulder = 0
        self._pcm_elbow = 0
        self._pcm_gripper = 0
        
        # Angle values for each joint
        self._angle_body = 0
        self._angle_neck_tilt = 0
        self._angle_neck_pan = 0
        self._angle_shoulder = 0
        self._angle_elbow = 0
        self._angle_gripper = 0

    def _update_pcm_positions(self):
        self._pcm_body = angle2pcm.body(self._angle_body)
        self._pcm_neck_tilt = angle2pcm.neck_tilt(self._angle_neck_tilt)
        self._pcm_neck_pan = angle2pcm.neck_pan(self._angle_neck_pan)
        self._pcm_shoulder = angle2pcm.shoulder(self._angle_shoulder)
        self._pcm_elbow = angle2pcm.elbow(self._angle_elbow)
        self._pcm_gripper = angle2pcm.gripper(self._angle_gripper)

    def _update_angle_positions(self):
        self._angle_body = pcm2angle.body(self._pcm_body)
        self._angle_neck_tilt = pcm2angle.head_tilt(self._pcm_neck_tilt)
        self._angle_neck_pan = pcm2angle.head_pan(self._pcm_neck_pan)
        self._angle_shoulder = pcm2angle.shoulder(self._pcm_shoulder)
        self._angle_elbow = pcm2angle.elbow(self._pcm_elbow)
        self._angle_gripper = pcm2angle.gripper(self._pcm_gripper)

    def get_jointstate(self):
        """
        Return necessary positions for publishing the joint states

        Returns
        --------
        positions_angle : list
            [body, neck_tilt, neck_pan, shoulder, elbow]
        """
        positions_angle = [self._angle_body, self._angle_neck_tilt, self._angle_neck_pan, self._angle_shoulder, self._angle_elbow]
        return positions_angle

    def get_gripper_open(self):
        """Returns the pcm value that corresponds to a fully opened gripper"""
        self._angle_gripper = 33
        self._update_pcm_positions()

        return self._pcm_gripper
    
    def get_gripper_close(self):
        """Returns the pcm value that corresponds to a fully closed gripper"""
        self._angle_gripper = 0
        self._update_pcm_positions()

        return self._pcm_gripper
    
    def get_arm_idle(self):
        """Returns the pcm values for [shoulder, elbow] for idle position in arms"""
        return np.array([angle2pcm.shoulder(pi/2), angle2pcm.elbow(-pi/2)])

    def get_arm_goto(self, coordinates: list):
        """
        Calculate eef position and return relevant angles for execution
        
        Parameters
        ----------
        coordinates : list
            A list of coordinates in [x, y, z] with float values

        Return
        ----------
        pcm_list : list
            A list of the pcm values for [body, shoulder, elbow]
        """

        inverse_kinematics = IK(coordinates[0], coordinates[1], coordinates[2])
        [self._angle_body, self._angle_shoulder, self._angle_elbow] = inverse_kinematics.angles

        self._update_pcm_positions()
        return [self._pcm_body, self._pcm_shoulder, self._pcm_elbow]

    
    def get_neck_tilt_down(self):
        """Returns the pcm value that corresponds to the angle of looking down towards the workspace"""
        return 1300 # Value is hardcoded
    

    def get_stance_first(self):
        """
        Returns a fully idle stance with 0 angles on all servos and an open gripper
        
        Return
        -------
        pcm_values : list
            [body, neck_tilt, neck_pan, shoulder, elbow, gripper]

        """
        [
            angle2pcm.body(),
            angle2pcm.neck_tilt(),
            angle2pcm.neck_pan(),
            angle2pcm.shoulder(),
            angle2pcm.elbow(),
            angle2pcm.gripper(33)
        ]

