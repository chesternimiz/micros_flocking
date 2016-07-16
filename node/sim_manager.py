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
    def __init__(self,r_id,send,pos_list):
        self._id=r_id
        #self._send=send
        self._publist=send
        self._pos_list=pos_list
    def __call__(self, msg):
        sendmsg=swarm.msg.Position()
        sendmsg.px=msg.pose.pose.position.x
        sendmsg.py=msg.pose.pose.position.y
        self._pos_list[self._id*2]=sendmsg.px
        self._pos_list[self._id*2+1]=sendmsg.py
        sendmsg.vx=msg.twist.twist.linear.x
        sendmsg.vy=msg.twist.twist.linear.y
        #self._send.send(sendmsg)
        self._publist[self._id].send(sendmsg)
        #print "rcv"+str(self._id)
        

class SimManager(FunctionUnit):
    def __init__(self, node_name):
        FunctionUnit.__init__(self, node_name)
        self._sub_list=[]
        self._pub_list=[]
        self._position_list=[]
        self._adj_list=[]
        self._adj_pub_list=[]

    def run(self):
        FunctionUnit.init_node(self)
        num = rospy.get_param('~robotnum',4)
        self._send = FunctionUnit.create_send(self, 'position_broadcast', swarm.msg.Broadcast)
        self._cb_list=[]
        for i in range(0,num):
           self._pub_list.append(FunctionUnit.create_send(self, 'robot_'+str(i)+'/position', swarm.msg.Position))
           self._position_list.append(-1)
           self._position_list.append(-1)  
           self._adj_list.append([])
           self._adj_pub_list.append(FunctionUnit.create_send(self, 'robot_'+str(i)+'/neighbor', swarm.msg.Neighbor))
        for i in range(0,num):
           self._cb_list.append(PosCallback(i,self._pub_list,self._position_list))
           self._sub_list.append(FunctionUnit.create_receive(self, "robot_"+str(i)+"/base_pose_ground_truth", Odometry, self._cb_list[i]))
        count = 1
        while not rospy.is_shutdown():
            time.sleep(0.1)
            for i in range(0,num):
                self._adj_list[i]=[]
            for i in range(0,num):
                 for j in range(i,num):
                     if self.calculate_dist(i,j) < 4 and self.calculate_dist(i,j)>0:
                         self._adj_list[i].append(j)
                         self._adj_list[j].append(i)
            for i in range(0,num):
                 neighbormsg=swarm.msg.Neighbor()
                 neighbormsg.data = self._adj_list[i]
                 self._adj_pub_list[i].send(neighbormsg)
            print "send loop send loop" + str(count)
            count +=1
        FunctionUnit.spin(self)
        
    def calculate_dist(self,i,j):
        result = pow(self._position_list[i*2]-self._position_list[j*2],2)+pow(self._position_list[i*2+1]-self._position_list[j*2+1],2)
        return pow(result,0.5)
'''
    def position_on_received(self,msg):
        sendmsg=swarm.msg.Broadcast()
        sendmsg.robot_ID=1
        sendmsg.odom=msg
        self._send.send(sendmsg)
'''

if __name__ == '__main__':
    try:
         rb = SimManager('sim_manager')
         rb.run()
    except rospy.ROSInterruptException:
         pass
