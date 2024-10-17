from hubert_kinematics.angle2pcm import Angle2pcm as _Angle2pcm
from hubert_kinematics.pcm2angle import Pcm2angle as _Pcm2angle
import numpy as np
from math import pi

angle2pcm = _Angle2pcm()
pcm2angle = _Pcm2angle()

class Hubert:
    def __init__(self):
        pass

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
        """Returns the pcm values for [shoulder joint, elbow joint] for idle position in arms"""
        return np.array([angle2pcm.shoulder(3*pi/2), angle2pcm.elbow(-pi/2)])
    
    def arm_lift():
        """DO NOT USE"""
        pass
    
    @staticmethod
    def head_tilt_down():
        """Returns the pcm value that corresponds to the angle of looking down towards the workspace"""
        return angle2pcm.head_tilt(1300)
    
