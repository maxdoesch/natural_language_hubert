from hubert_kinematics.angle2pcm import Angle2pcm as _Angle2pcm
from hubert_kinematics.pcm2angle import Pcm2angle as _Pcm2angle
import numpy as np
from math import pi

from hubert_kinematics.ik_solver import InverseKinematics as IK

angle2pcm = _Angle2pcm()
pcm2angle = _Pcm2angle()

class Hubert:
    def __init__(self):
        """
        Hubert virtual stance class
        Gives pcm values for given tasks. Is used to store and get every joint value for Hubert

        get_stance - Considers every joint
        get_arm - Considers the body, shoulder and elbow joints
        get_gripper - Considers only the gripper joint
        get_neck - Considers the two neck joints
        get_jointstate - Returns list for publishing jointstate
        """

        # Angle values for each joint
        self._angle_body = 0
        self._angle_neck_tilt = 0
        self._angle_neck_pan = 0
        self._angle_shoulder = 0
        self._angle_elbow = 0
        self._angle_gripper = 33

        # PCM values for each joint (Pass through the update method)
        self._update_angle2pcm_positions()

    def _update_angle2pcm_positions(self):
        self._pcm_body = angle2pcm.body(self._angle_body)
        self._pcm_neck_tilt = angle2pcm.neck_tilt(self._angle_neck_tilt)
        self._pcm_neck_pan = angle2pcm.neck_pan(self._angle_neck_pan)
        self._pcm_shoulder = angle2pcm.shoulder(self._angle_shoulder)
        self._pcm_elbow = angle2pcm.elbow(self._angle_elbow)
        self._pcm_gripper = angle2pcm.gripper(self._angle_gripper)

        return [
            self._pcm_body,
            self._pcm_neck_tilt,
            self._pcm_neck_pan,
            self._pcm_shoulder,
            self._pcm_elbow,
            self._pcm_gripper
        ]

    def _update_pcm2angle_positions(self):
        self._angle_body = np.round(pcm2angle.body(self._pcm_body), 2)
        self._angle_neck_tilt = np.round(pcm2angle.neck_tilt(self._pcm_neck_tilt), 2)
        self._angle_neck_pan = np.round(pcm2angle.neck_pan(self._pcm_neck_pan), 2)
        self._angle_shoulder = np.round(pcm2angle.shoulder(self._pcm_shoulder), 2)
        self._angle_elbow = np.round(pcm2angle.elbow(self._pcm_elbow), 2)
        self._angle_gripper = np.round(pcm2angle.gripper(self._pcm_gripper), 2)

        return [
            self._angle_body,
            self._angle_neck_tilt,
            self._angle_neck_pan,
            self._angle_shoulder,
            self._angle_elbow,
            self._angle_gripper
        ]

    def get_jointstate(self):
        """
        Return necessary positions for publishing the joint states

        Return
        --------
        positions_angle : list
            [body, neck_tilt, neck_pan, shoulder, elbow]
        """
        positions_angle = [self._angle_body, self._angle_neck_tilt, self._angle_neck_pan, self._angle_shoulder, self._angle_elbow]
        return positions_angle

    def get_gripper_open(self):
        """
        Fully open gripper
        
        Return
        --------
        pcm_value : int
            The value for 'gripper'
        """
        self._angle_gripper = 33
        self._update_angle2pcm_positions()

        return self._pcm_gripper
    
    def get_gripper_close(self):
        """
        Fully closed gripper

        Return
        --------
        pcm_value : int
            The value for 'gripper'
        """
        self._angle_gripper = 0
        self._update_angle2pcm_positions()

        return self._pcm_gripper
    
    def get_arm_idle(self):
        """
        Idle arm stance. Upper arm horizontal and lower arm straight down.
        
        Return
        ----------
        pcm_list : list
            A list of the pcm values for [shoulder, elbow]
        """
        self._angle_shoulder = pi/2
        self._angle_elbow = -pi/2
        self._update_angle2pcm_positions()

        return [self._pcm_shoulder, self._pcm_elbow]

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

        print(coordinates)

        self._update_angle2pcm_positions()
        return [self._pcm_body, self._pcm_shoulder, self._pcm_elbow]

    
    def get_neck_tilt_down(self):
        """
        Looking down position for the neck

        Return
        ----------
        pcm_value : int
            The value for 'neck_tilt'
        """
        self._pcm_neck_tilt = 1300 # Value is hardcoded
        self._update_pcm2angle_positions()
        print(self._angle_neck_tilt)

        return self._pcm_neck_tilt

    def get_cam_move_right(self, value=200):
        
        if self._pcm_neck_pan > 1840 and self._pcm_body < 1700:
            self._pcm_body += value
        elif self._pcm_body > 1700:
            self._pcm_neck_pan = 550
            self._pcm_body = self.get_body_forward()
        else:
            self._pcm_neck_pan += value
        self._update_pcm2angle_positions()

        return self._pcm_neck_pan, self._pcm_body

    def get_stance_first(self):
        """
        First stance. Hubert facing right with arms down and head straight
        
        Return
        -------
        pcm_values : list
            [body, neck_tilt, neck_pan, shoulder, elbow, gripper]

        """
        self._angle_body = 0
        self._angle_neck_tilt = 0
        self._angle_neck_pan = 0
        self._angle_shoulder = 0
        self._angle_elbow = 0
        self._angle_gripper = 33

        pcm_values = self._update_angle2pcm_positions()
        
        return pcm_values

    def get_body_forward(self):
        """
        The body should just be facing forward. No other changes
        
        Return
        --------
        pcm_body_value : int
        """
        self._angle_body = 0

        self._update_angle2pcm_positions()
        pcm_value = self._pcm_body

        return pcm_value

hub = Hubert()
print(hub.get_stance_first())