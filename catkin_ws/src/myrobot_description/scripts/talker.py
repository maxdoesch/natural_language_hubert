#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import UInt16
import time

# Body = 560 - 2330
# HeadPan = 550 - 2340
# HeadTilt = 950 - 2400
# Shoulder = 750 - 2200
# Elbow = 550 - 2400
# Gripper = 550 - 2150


def body_joint_talker():
    pub1 = rospy.Publisher('/servo_body', UInt16, queue_size=10)
    #pub2 = rospy.Publisher('/joint_states')

    rospy.init_node('body_joint_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    z_mean_value = 1500
    rospy.loginfo(z_mean_value)
    pub1.publish(z_mean_value)

    z_next_value = 1700
    while not rospy.is_shutdown():
        if z_next_value > 2350:
            z_next_value = 500

        rospy.loginfo(z_next_value)
        pub1.publish(z_next_value)

        z_next_value = z_next_value + 200
        time.sleep(2)
        rate.sleep()

if __name__ == '__main__':
    try:
        body_joint_talker()
    except rospy.ROSInterruptException:
        pass