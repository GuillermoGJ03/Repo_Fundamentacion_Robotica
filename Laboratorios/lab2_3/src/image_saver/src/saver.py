#! /usr/bin/env python

import rospy 
from std_msgs.msg import String, UInt8
from sensor_msgs.msg import Image 
import cv2
from cv_bridge import CvBridge

class Saver:
    def __init__(self):
        rospy.init_node('image_saver')
        
        rospy.Subscriber('/counter', UInt8, self.counter_callback)
        rospy.Subscriber('/hour_getter', String, self.hour_callback)
        rospy.Subscriber('/cam/image_raw', Image, self.camera_callback)

        rospy.Timer(rospy.Duration(5), self.timer_callback)

        self.counter = 0
        self.hour = ''
        self.img = None

        self.bridge = CvBridge()

    def counter_callback(self, msg):
        self.counter =msg.data

    def hour_callback(self,msg):
        self.hour = msg.data
    
    def camera_callback(self,msg):
        self.img=self.bridge.imgmsg_to_cv2(msg,'bgr8')

    def timer_callback(self, timer):
        cv2.imwrite('/home/guillermo/imgs/img_{}_{}.jpg'.format(self.hour,self.counter),self.img)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    s= Saver()
    s.run()




