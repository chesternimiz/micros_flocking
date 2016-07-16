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
class RobotDetect(FunctionUnit):
    def __init__(self, node_name):
        FunctionUnit.__init__(self, node_name)
        self._x = 0.0
        self._y = 0.0
        self._sphere = 1.0
        self._min_range = 0.3
        self._max_speed = 1.0
 
    def run(self):
        FunctionUnit.init_node(self)
        self._id = rospy.get_param('~robot_id',0)
        self._robotnum = rospy.get_param('~robot_num',0)
        self._robot_list = []
        for i in range (0,2*self._robotnum):
           self._robot_list.append(0.0)
        self._send = FunctionUnit.create_send(self, 'robot_detect', Twist)
        self._rcv = FunctionUnit.create_receive(self, "/position_broadcast", swarm.msg.Broadcast, self.broadcast_on_received)
        thread.start_new_thread(self.start_detect,())
        FunctionUnit.spin(self)

    def broadcast_on_received(self,msg):
        if msg.robot_ID == self._id:
           self._x = msg.x
           self._y = msg.y
        else:
           self._robot_list[2*msg.robot_ID] = self._x - msg.x
           self._robot_list[2*msg.robot_ID+1] = self._y - msg.y
    def start_detect(self):
        while not rospy.is_shutdown():
            time.sleep(0.1)
            msg = Twist()
            for i in range (0,self._robotnum):
                vector_x = self._robot_list[2*i]
                vector_y = self._robot_list[2*i+1]
                dist = pow(vector_x*vector_x+vector_y*vector_y,0.5)
                if dist >= self._sphere or dist ==0:
                    continue
                if dist <= self._min_range:
                    scale = dist / self._max_speed
                    msg.linear.x += vector_x /scale
                    msg.linear.y += vector_y / scale
                else:
                    desired_speed = (self._sphere - dist)/(self._sphere - self._min_range) * self._max_speed
                    msg.linear.x += vector_x / dist * desired_speed
                    msg.linear.y += vector_y / dist * desired_speed
            speed = pow(msg.linear.x*msg.linear.x+msg.linear.y*msg.linear.y,0.5)
            scale = speed / self._max_speed
            if scale < 1:
                scale = 1
            msg.linear.x /= scale
            msg.linear.y /= scale
            self._send.send(msg)

if __name__ == '__main__':
    try:
         rd = RobotDetect('robot_detect')
         rd.run()
    except rospy.ROSInterruptException:
         pass
