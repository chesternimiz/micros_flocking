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
class PosCallback(object):
    def __init__(self,r_id,send):
        self._id=r_id
        self._send=send
        #self._publist=publist
    def __call__(self, msg):
        sendmsg=swarm.msg.Broadcast()
        sendmsg.robot_ID=self._id
        sendmsg.x=msg.pose.pose.position.x
        sendmsg.y=msg.pose.pose.position.y
        self._send.send(sendmsg)
        #self._publist[self._id].send(sendmsg)
        

class RobotBroadcaster(FunctionUnit):
    def __init__(self, node_name):
        FunctionUnit.__init__(self, node_name)
        self._sub_list=[]
        #self._pub_list=[]

    def run(self):
        FunctionUnit.init_node(self)
        num = rospy.get_param('~robotnum',4)
        self._send = FunctionUnit.create_send(self, 'position_broadcast', swarm.msg.Broadcast)
        self._cb_list=[]
        #for i in range(0,num):
        #   self._pub_list.append(FunctionUnit.create_send(self, 'robot_'+str(i)+'/position', swarm.msg.Broadcast))  
        for i in range(0,num):
           self._cb_list.append(PosCallback(i,self._send))
           self._sub_list.append(FunctionUnit.create_receive(self, "robot_"+str(i)+"/base_pose_ground_truth", Odometry, self._cb_list[i]))
        FunctionUnit.spin(self)

    def position_on_received(self,msg):
        sendmsg=swarm.msg.Broadcast()
        sendmsg.robot_ID=1
        sendmsg.odom=msg
        self._send.send(sendmsg)


if __name__ == '__main__':
    try:
         rb = RobotBroadcaster('robot_position_broadcaster')
         rb.run()
    except rospy.ROSInterruptException:
         pass
