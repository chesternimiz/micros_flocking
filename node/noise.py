#!/usr/bin/env python

__author__ = "Zhongxuan Cai"

import sys
sys.path.append("..")
from alliance.function_unit import FunctionUnit
import actionlib
import multi_robot_patrol.msg
import rospy
from geometry_msgs.msg import Twist
import random

class Noise(FunctionUnit):
    def __init__(self, node_name):
        FunctionUnit.__init__(self, node_name)
    #rospy.get_param must be used after init_node!
        
    def run(self):
        FunctionUnit.init_node(self)
        self._low=rospy.get_param('~low',0)
        self._up=rospy.get_param('~up',0)
        self._rate=rospy.Rate(10)
        self._noise_send = FunctionUnit.create_send(self, 'cmd_vel', Twist)
        while not rospy.is_shutdown():
            msg = Twist()
            msg.linear.x=random.uniform(self._low,self._up)
            msg.linear.y=random.uniform(self._low,self._up)
            self._noise_send.send(msg)
            self._rate.sleep()

if __name__ == '__main__':
    try:
         noise = Noise('r0_noise')
         noise.run()
    except rospy.ROSInterruptException:
         pass
       
