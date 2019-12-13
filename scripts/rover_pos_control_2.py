#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 22 12:29:26 2019

@author: jogan
"""
import rospy
import sys
from geometry_msgs.msg import Twist, Point

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

tar_pos = sys.argv[1]   #Target-positon

class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.publisher = rospy.Publisher('/rover_twist', Twist, queue_size=10)

        # messageの型を作成
        self.twistmsg = Twist()

    def make_msg(self, velocity):
        # 処理を書く
        self.twistmsg.linear.x = 0
        self.twistmsg.linear.y = 0
        self.twistmsg.linear.z = velocity
        self.twistmsg.angular.x = 0
        self.twistmsg.angular.y = 0
        self.twistmsg.angular.z = 0
        

    def send_msg(self):
        # messageを送信
        #self.make_msg()
        #self.publisher.publish(self.twistmsg)
        print self.twistmsg


class Subscribe_Publishers():
    def __init__(self, pub):
        # Subscriberを作成
        self.subscriber = rospy.Subscriber('/cylinder_pos', Point, self.callback, queue_size=10)
            # messageの型を作成
        self.pointmsg = Point()
        self.velocity = 0.0 #Velocity of Linear motion joint
        self.dif = 0        #Difference between target-position and observed-position
        self.pub = pub

    def callback(self, pointmsg):
        #ここに位置制御の仕方を書く
        print pointmsg.z
        self.dif = float(tar_pos) - pointmsg.z
        if -1 <= self.dif <= 1:
            self.velocity = 0
        else:
            self.velocity = (self.dif*0.0005)
        
        # callback時の処理
        self.pub.make_msg(self.velocity)
        # publish
        self.pub.send_msg()


def main():
    # nodeの立ち上げ
    rospy.init_node('cylinder_position_controller', anonymous=True, disable_signals=True)

    # クラスの作成
    pub = Publishsers()
    sub = Subscribe_Publishers(pub)
    
    rospy.spin()
    
    rate = rospy.Rate(10)

if __name__ == '__main__':
   try:
        main()
   except KeyboardInterrupt: #Ctrl-c
        print "--- Stop by Ctrl+c ---"
        mySigintHandler()
   except rospy.ROSInterruptException: #other exception?
        print "bye"