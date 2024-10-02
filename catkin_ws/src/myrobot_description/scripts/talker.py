#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import UInt16

def talker():
    pub = rospy.Publisher('/servo_elbow', UInt16, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    minmax = [1000, 2000]
    while not rospy.is_shutdown():
        z_int = int(input(f"Enter z value for the elbow (between {minmax[0]} and {minmax[1]}): "))
        if minmax[0] <= z_int <= minmax[1]:
            rospy.loginfo(z_int)
            pub.publish(z_int)
        else:
            break

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass