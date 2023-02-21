#! /usr/bin/env python

import rospy
import datetime
from std_msgs.msg import String

if __name__ == "__main__":
    pub = rospy.Publisher('hour_getter', String, queue_size=10)

    rospy.init_node('hour_getter_node')
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        msg = datetime.datetime.now()
        msg= "{ano}{mes}{dia}{hora}{minuto}_{segundo}".format(dia=msg.day, mes=msg.month, ano=msg.year, hora=msg.hour, minuto=msg.minute, segundo=msg.second)
        pub.publish(msg)
        rate.sleep()