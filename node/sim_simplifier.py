#!/usr/bin/env python

__author__ = "Zhongxuan Cai"

import sys
sys.path.append("..")
from alliance.function_unit import FunctionUnit
#from function_unit import FunctionUnit
import actionlib
import swarm.msg
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import thread
class SimSimplifier(FunctionUnit):
    def __init__(self, node_name):
        FunctionUnit.__init__(self, node_name)
        #self._pub_list=[]

    def run(self):
        FunctionUnit.init_node(self)
        self._send = FunctionUnit.create_send(self, 'position', swarm.msg.Broadcast)
        self._rcv=FunctionUnit.create_receive(self, "base_pose_ground_truth", Odometry, self.odom_on_received)
        FunctionUnit.spin(self)

    def odom_on_received(self,msg):
        sendmsg=swarm.msg.Broadcast()
        sendmsg.robot_ID=1
        sendmsg.x = msg.pose.pose.position.x
        sendmsg.y = msg.pose.pose.position.y
        self._send.send(sendmsg)


if __name__ == '__main__':
    try:
         rb = SimSimplifier('robot_sim')
         rb.run()
    except rospy.ROSInterruptException:
         pass
