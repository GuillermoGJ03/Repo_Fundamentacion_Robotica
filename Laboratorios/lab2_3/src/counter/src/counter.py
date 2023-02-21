#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt8



if __name__ == '__main__':
    pub = rospy.Publisher('counter', UInt8, queue_size=10)
    rospy.init_node('counter_node')
    msg = 0
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish(msg)

        if msg == 255:
            msg = 0
        else:
            msg += 1
    
        rate.sleep()