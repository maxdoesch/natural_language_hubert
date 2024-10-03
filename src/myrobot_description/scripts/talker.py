#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import String
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

#from pcm2angle import body

# Body = 560 - 2330
# HeadPan = 550 - 2340
# HeadTilt = 950 - 2300
# Shoulder = 750 - 2200
# Elbow = 550 - 2400
# Gripper = 550 - 2150

# Right angle value for the camera: 1300

# Always take robot's movements time into consideration

# Define pi as PI
PI = 3.1415926536

def __calculate(pcm, angle_min, angle_max, pcm_min, pcm_max) -> float:
    # Raise error if PCM is outside of the boundary
    if pcm < pcm_min or pcm > pcm_max:
        raise ValueError(f"PCM must be between {pcm_min} and {pcm_max}.")

    # Linear interpolation from PCM to angle
    angle_value = angle_min + (pcm - pcm_min) * (angle_max - angle_min) / (pcm_max - pcm_min)

    return angle_value  # Return the angle as a float

def pcm2angle_body(pcm: int) -> float:
    """
    Function that converts PCM for BODY servo to angle.
    
    :param pcm: PCM value [560, 2330]
    :return: Corresponding angle in radians.
    """

    angle_min = -PI/2
    angle_max = PI/2

    pcm_min = 560
    pcm_max = 2330

    angle_value = __calculate(pcm, angle_min, angle_max, pcm_min, pcm_max)

    return angle_value  # Return as a float

def pcm2angle_head_pan(pcm: int) -> float:
    """
    Function that converts PCM for HEAD_PAN servo to angle.
    
    :param pcm: PCM value [550, 2400]
    :return: Corresponding angle in radians.
    """

    angle_min = -PI/2
    angle_max = PI/2

    pcm_min = 550
    pcm_max = 2400

    angle_value = __calculate(pcm, angle_min, angle_max, pcm_min, pcm_max)

    return angle_value  # Return as a float

def pcm2angle_head_tilt(pcm: int) -> float:
    """
    Function that converts PCM for HEAD_TILT servo to angle.
    
    :param pcm: PCM value [950, 2400]
    :return: Corresponding angle in radians.
    """

    angle_min = -PI/4
    angle_max = PI/2

    pcm_min = 950
    pcm_max = 2400

    angle_value = __calculate(pcm, angle_min, angle_max, pcm_min, pcm_max)

    return angle_value  # Return as a float

def pcm2angle_shoulder(pcm: int) -> float:
    """
    Function that converts PCM for SHOULDER servo to angle.
    
    :param pcm: PCM value [750, 2200]
    :return: Corresponding angle in radians.
    """

    angle_min = -PI/6
    angle_max = PI/2

    pcm_min = 750
    pcm_max = 2200

    angle_value = __calculate(pcm, angle_min, angle_max, pcm_min, pcm_max)

    return angle_value  # Return as a float

def pcm2angle_elbow(pcm: int) -> float:
    """
    Function that converts PCM for ELBOW servo to angle.
    
    :param pcm: PCM value [550, 2400]
    :return: Corresponding angle in radians.
    """

    angle_min = -PI/2
    angle_max = PI/2

    pcm_min = 550
    pcm_max = 2400

    angle_value = __calculate(pcm, angle_min, angle_max, pcm_min, pcm_max)

    return angle_value  # Return as a float

def create_joint_state_msg(positions):
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['base_joint', 'neck_joint', 'cam_joint', 'shoulder_joint', 'elbow_joint']
    msg.position = positions
    msg.velocity = []
    msg.effort = []

    return msg

def coordinates_callback(data):

    global coordinates_received

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    coordinates_received = True

def label_callback(data):

    global label_recieved

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    label_recieved = True

def coordinates_listener():

    rospy.init_node('coordinates_listener', anonymous=True)

    rospy.Subscriber("/coordinates", Point, coordinates_callback)

    rospy.spin()

def label_listener():

    rospy.init_node('label_listener', anonymous=True)

    rospy.Subscriber("/label_topic", Point, label_callback)

    rospy.spin()


def joints_talker():

    pub_body = rospy.Publisher('/servo_body', UInt16, queue_size=10)

    pub_neck_tilt = rospy.Publisher('/servo_neck_tilt', UInt16, queue_size=10)

    pub_elbow = rospy.Publisher('/servo_elbow', UInt16, queue_size=10)

    pub_joint_states = rospy.Publisher('/joint_states', JointState, queue_size=10)

    rospy.init_node('body_joint_talker', anonymous=True)
    #rospy.init_node('neck_tilt_joint_talker', anonymous=True) # Apparently not needed, but I don't see why 
    rate = rospy.Rate(10) # 10hz

    z_next_value = 1450 # Mean value would actually be 1425
    neck_tilt_value = 1300 
    rospy.loginfo(z_next_value)
    pub_body.publish(z_next_value)
    rospy.loginfo(neck_tilt_value)
    pub_neck_tilt.publish(neck_tilt_value)

    positions = [pcm2angle_body(z_next_value), 0, pcm2angle_head_tilt(neck_tilt_value), 0, 0]
    msg = create_joint_state_msg(positions)
    pub_joint_states.publish(msg)

    time.sleep(5)
    pub_joint_states.publish(msg)

    elbow_value = 1400

    while not rospy.is_shutdown():
        while not coordinates_received:

            z_next_value = z_next_value + 200

            if z_next_value > 2330:
                z_next_value = 560

            rospy.loginfo(z_next_value)
            pub_body.publish(z_next_value)

            positions = [pcm2angle_body(z_next_value), 0, 0, 0, 0]
            msg = create_joint_state_msg(positions)
            pub_joint_states.publish(msg)

            if z_next_value == 560:
                time.sleep(5)
                pub_joint_states.publish(msg)

            time.sleep(2)
            pub_joint_states.publish(msg)
            rate.sleep()
            pub_joint_states.publish(msg)

        if coordinates_received:
            if elbow_value != 2200:
                elbow_value = 2200
                rospy.loginfo(elbow_value)
                pub_elbow.publish(elbow_value)

                positions = [0, 0, 0, 0, pcm2angle_elbow(elbow_value)]
                msg = create_joint_state_msg(positions)
                pub_joint_states.publish(msg)

if __name__ == '__main__':
    try:
        joints_talker()
        label_listener()
        coordinates_listener()
    except rospy.ROSInterruptException:
        pass