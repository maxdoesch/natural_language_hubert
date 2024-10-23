#!/usr/bin/env python3
# license removed for brevity
import rospy
import copy
from std_msgs.msg import UInt16
from std_msgs.msg import String
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

from hubert_launch.srv import GoToCoordinate, MoveArm
from std_srvs.srv import Empty

from hubert_kinematics.angle2pcm import Angle2pcm as _Angle2pcm
from hubert_actions import Hubert as _Hubert
from hubert_kinematics.pcm2angle import Pcm2angle as _Pcm2angle


from hubert_kinematics.ik_solver import ForwardKinematics as FK
from hubert_kinematics.ik_solver import InverseKinematics as IK
from math import pi as PI
import random
import numpy as np

angle2pcm = _Angle2pcm()
pcm2angle = _Pcm2angle()
hubert = _Hubert()

class HubertListener:
    def __init__(self):
        # Service
        self.srv_goto = rospy.Service('/hubert/go_to_coordinate', GoToCoordinate, self.arm_goto)
        self.srv_open_eef = rospy.Service('/hubert/open_effector', Empty, self.open_effector)
        self.srv_grab = rospy.Service('/hubert/grab', Empty, self.grab)
        self.srv_move_arm = rospy.Service('/hubert/move_arm', MoveArm, self.move_arm)
        self.srv_idle = rospy.Service('/hubert/idle', Empty, self.idle)

        # Publishers
        self.pub_body = rospy.Publisher('/servo_body', UInt16, queue_size=10, latch=True)
        self.pub_neck_tilt = rospy.Publisher('/servo_neck_tilt', UInt16, queue_size=10, latch=True)
        self.pub_neck_pan = rospy.Publisher('/servo_neck_pan', UInt16, queue_size=10, latch=True)
        self.pub_shoulder = rospy.Publisher('/servo_shoulder', UInt16, queue_size=10)
        self.pub_elbow = rospy.Publisher('/servo_elbow', UInt16, queue_size=10)
        self.pub_gripper = rospy.Publisher('/servo_gripper', UInt16, queue_size=10)
        self.pub_joint_states = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # Local variables
        self.old_coordinates = [0, 0, 0]
        self.x_offset = 0.00
        self.y_offset = 0.01
        self.z_offset = .04

        # Rospy node
        rospy.init_node('joints_talker', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz


    @staticmethod
    def create_joint_state_msg(positions):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ['base_joint', 'cam_joint', 'neck_joint', 'shoulder_joint', 'elbow_joint']
        msg.position = positions
        msg.velocity = []
        msg.effort = []

        return msg

    def publish(self, pcm_value, publisher, joint_states_publisher, delay=2):
        publisher.publish(pcm_value)
        joint_states_publisher.publish(self.create_joint_state_msg(hubert.get_jointstate()))
        time.sleep(delay)
        
    
    def arm_goto(self, coordinate):
        coordinates = [coordinate.point.x + self.x_offset, coordinate.point.y + self.y_offset, coordinate.point.z + self.z_offset]
        [body_value, shoulder_value, elbow_value] = hubert.get_arm_goto(coordinates)

        self.publish(body_value, self.pub_body, self.pub_joint_states)
        self.publish(elbow_value, self.pub_elbow, self.pub_joint_states)
        self.publish(shoulder_value, self.pub_shoulder, self.pub_joint_states)

        self.old_coordinates = coordinates
        
        time.sleep(3)
        print("Arm go to coordinate done!")

        return True
    
    def open_effector(self):
        gripper_value = hubert.get_gripper_open()

        self.publish(gripper_value, self.pub_gripper, self.pub_joint_states)

        print("End effector is open!")
    
    def grab(self):
        # Go down
        coordinates = self.old_coordinates - [0, 0, self.z_offset]

        [body_value, shoulder_value, elbow_value] = hubert.get_arm_goto(coordinates)

        self.publish(body_value, self.pub_body, self.pub_joint_states)
        self.publish(elbow_value, self.pub_elbow, self.pub_joint_states)
        self.publish(shoulder_value, self.pub_shoulder, self.pub_joint_states)

        # Grab
        gripper_value = hubert.get_gripper_close()
        self.publish(gripper_value, self.pub_gripper, self.pub_joint_states, delay=3)

        # Go up
        coordinates = self.old_coordinates

        [body_value, shoulder_value, elbow_value] = hubert.get_arm_goto(coordinates)

        self.publish(body_value, self.pub_body, self.pub_joint_states)
        self.publish(elbow_value, self.pub_elbow, self.pub_joint_states)
        self.publish(shoulder_value, self.pub_shoulder, self.pub_joint_states)

        print("Hubert grabbed object!")


    def move_arm(self, relative_coordinates):
        coordinates = self.old_coordinates + [relative_coordinates.x, relative_coordinates.y, 0]

        [body_value, shoulder_value, elbow_value] = hubert.get_arm_goto(coordinates)

        positions = hubert.get_jointstate()
        msg = self.create_joint_state_msg(positions)
        self.pub_joint_states.publish(msg)

        self.publish(body_value, self.pub_body, self.pub_joint_states)
        self.publish(elbow_value, self.pub_elbow, self.pub_joint_states)
        self.publish(shoulder_value, self.pub_shoulder, self.pub_joint_states)

        self.old_coordinates = coordinates
        
        time.sleep(3)
        print("Move arm relative to coordinate done!")

        return True
    
    def idle(self):
        [body_value, neck_tilt_value, neck_pan_value, shoulder_value, elbow_value, gripper_value] = hubert.get_stance_first()

        self.pub_shoulder.publish(shoulder_value)
        time.sleep(1)
        self.pub_elbow.publish(elbow_value)
        time.sleep(1)
        self.pub_neck_tilt.publish(neck_tilt_value)
        time.sleep(1)
        self.pub_neck_pan.publish(neck_pan_value)
        time.sleep(1)
        self.pub_body.publish(body_value)
        time.sleep(1)
        self.pub_gripper.publish(gripper_value)

        angles = hubert.get_jointstate()
        msg = self.create_joint_state_msg(angles)
        self.pub_joint_states.publish(msg)

        print("Idle position done!")

    def run_start(self):
        print("Starting up...")

        [body_value, neck_tilt_value, neck_pan_value, shoulder_value, elbow_value, gripper_value] = hubert.get_stance_first()
        
        self.pub_shoulder.publish(shoulder_value)
        time.sleep(1)
        self.pub_elbow.publish(elbow_value)
        time.sleep(1)
        self.pub_body.publish(body_value)
        time.sleep(1)
        self.pub_neck_tilt.publish(neck_tilt_value)
        time.sleep(1)
        self.pub_neck_pan.publish(neck_pan_value)
        time.sleep(1)
        self.pub_body.publish(body_value)
        time.sleep(1)
        self.pub_gripper.publish(gripper_value)

        #Publish on joint states

        time.sleep(4)
        print("First stance done!")

        [shoulder_value, elbow_value] = hubert.get_arm_idle()
        self.pub_elbow.publish(elbow_value)
        time.sleep(1)
        self.pub_shoulder.publish(shoulder_value)
        
        print("Idle arms done!")
        time.sleep(4)

        self.pub_body.publish(body_value)
        time.sleep(1)

        neck_tilt_value = hubert.get_neck_tilt_down()
        self.pub_neck_tilt.publish(neck_tilt_value)
        time.sleep(4)
        print("Tilt neck done!")

        angles = [pcm2angle.body(body_value), pcm2angle.neck_tilt(neck_tilt_value), pcm2angle.neck_pan(neck_pan_value),
                        pcm2angle.shoulder(shoulder_value), pcm2angle.elbow(elbow_value)]
        
        print("Published joint states!")
        while True:
            angles = hubert.get_jointstate()
            msg = self.create_joint_state_msg(angles)
            self.pub_joint_states.publish(msg)


def main():

    hu_listener = HubertListener()
    hu_listener.run_start()


if __name__ == '__main__':
    main()