#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 21 20:21:30 2019

@author: jogan
"""

import rospy
import sys

from silva_core.msg import Evans
import modules.topics as topics
import modules.utils as utils

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

tar_pos = 0 #sys.argv[1]
direc = 0

K = 0.002 

def mySigintHandler():
    pub = rospy.Publisher('/rover_twist', Twist, queue_size=10)
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0

    for i in range(10):
        pub.publish(msg)

        
def callback(msg):
    dif = float(tar_pos) - msg.z
    global direc
    if -1 <= dif <= 1:
        direc = 0
        return
    else:
        direc = (dif*K)
        
def callback_2(msg):
    global tar_pos
    tar_pos = msg.payload[14]
    
def move():
    pub = rospy.Publisher('/rover_twist', Twist, queue_size=10)
    msg = Twist()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg.linear.x = 0#0.2
        msg.linear.y = 0
        msg.linear.z = direc
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        pub.publish(msg)
        #print msg
        rate.sleep()



def start():
    rospy.init_node('cylinder_position_controller', anonymous=True, disable_signals=True)

    sub_2 = rospy.Subscriber(topics.com['mixer'], Evans, callback_2)
    sub = rospy.Subscriber('/cylinder_pos', Point, callback, queue_size=10)

    move()

if __name__ == '__main__':
    try:
        start()
    except KeyboardInterrupt: #Ctrl-c
        print "--- Stop by Ctrl+c ---"
        mySigintHandler()
    except rospy.ROSInterruptException: #other exception?
        print "bye"
