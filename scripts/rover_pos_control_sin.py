#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 21 18:13:09 2019

@author: jogan
"""

import rospy
import sys
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import math,time

f_time = time.time()
tar_pos = 0
direc = 0

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
    global tar_pos,start
    tar_pos = 50*math.sin(2*(f_time - time.time())*math.pi*134./60*0.8)-100
    dif = float(tar_pos) - msg.z
    global direc
    if -2 <= dif <= 2:
        direc = 0
        return
    else:
        direc = (dif*0.001)

def move():
    pub = rospy.Publisher('/rover_twist', Twist, queue_size=10)
    msg = Twist()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = direc
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        pub.publish(msg)
        print msg
        rate.sleep()



def start():
    rospy.init_node('cylinder_position_controller', anonymous=True, disable_signals=True)
    
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