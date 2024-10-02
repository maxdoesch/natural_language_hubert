#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import UInt16
import time
from sensor_msgs.msg import JointState

# Body = 560 - 2330
# HeadPan = 550 - 2340
# HeadTilt = 950 - 2300
# Shoulder = 750 - 2200
# Elbow = 550 - 2400
# Gripper = 550 - 2150

# Right angle value for the camera: 1300

# Always take robot's movements time into consideration

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
    time.sleep(5)

    detection = 0
    elbow_value = 1400

    while not rospy.is_shutdown():
        while detection < 9:

            z_next_value = z_next_value + 200

            if z_next_value > 2330:
                z_next_value = 560

            rospy.loginfo(z_next_value)
            pub_body.publish(z_next_value)
            if z_next_value == 560:
                time.sleep(5)

            time.sleep(2)
            rate.sleep()

            detection += 1

        if detection == 9:
            if elbow_value != 2200:
                elbow_value = 2200
                rospy.loginfo(elbow_value)
                pub_elbow.publish(elbow_value)

if __name__ == '__main__':
    try:
        joints_talker()
    except rospy.ROSInterruptException:
        pass