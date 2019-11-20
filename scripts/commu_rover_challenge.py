#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its[ 2636.749729] [11-08 09:18:25.398] wl_notify_connect_status: wl_bss_connect_done succeeded with 4c:e6:76:ac:71:63

#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
#from std_msgs.msg import String
from geometry_msgs.msg import Twist

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


def talker():
    pub = rospy.Publisher('/rover_twist', Twist, queue_size=10)
    rospy.init_node('random_husky_commands', anonymous=True, disable_signals=True)
    msg = Twist()
    rate = rospy.Rate(2) # 10hz
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = -0.04
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0
    while not rospy.is_shutdown():
#        msg.linear.z *= -1
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except KeyboardInterrupt: #Ctrl-c
        print "--- Stop by Ctrl+c ---"
        mySigintHandler()
    except rospy.ROSInterruptException: #other exception?
        print "bye"
    
