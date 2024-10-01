#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import UInt16

def talker():
    pub = rospy.Publisher('/servo_elbow', UInt16, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        z_int = int(input("Enter z value for the elbow (between 1000 and 2000?)"))
        rospy.loginfo(z_int)
        pub.publish(z_int)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass