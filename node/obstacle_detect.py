#!/usr/bin/env python

__author__ = "Zhongxuan Cai"

import sys
sys.path.append("..")
from alliance.function_unit import FunctionUnit
#from function_unit import FunctionUnit
import actionlib
import multi_robot_patrol.msg
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import thread
import swarm.msg

class ObstacleDetect(FunctionUnit):
    def __init__(self, node_name):
        FunctionUnit.__init__(self, node_name)
        #self._robot=robot_name
        self._x = 0.0#robot_x
        self._y = 0.0#robot_y
        self._vector_x = 0.0#vector from obstacle to robot
        self._vector_y = 0.0

    def run(self):
        FunctionUnit.init_node(self)
        position_receive = FunctionUnit.create_receive(self, "position", swarm.msg.Broadcast, self.position_on_received)
        self._detect_send = FunctionUnit.create_send(self, 'obstacle_detection', Twist)
        thread.start_new_thread(self.start_detect,())
        FunctionUnit.spin(self)

    def position_on_received(self,msg):
        self._x = msg.x
        self._y = msg.y

    def detect(self,minX,minY,maxX,maxY):
        if self._x < minX:
            self._vector_x = self._x - minX
        else: 
            if self._x > maxX:
               self._vector_x = self._x - maxX
            else:
               self._vector_x =0
        if self._y < minY:
            self._vector_y = self._y - minY
        else: 
            if self._y > maxY:
               self._vector_y = self._y - maxY
            else:
               self._vector_y =0

    def start_detect(self):
         while not rospy.is_shutdown():
            time.sleep(0.1)
            self.detect(4.39,6.93,5.67,8.18)
            msg = Twist()
            msg.linear.x=self._vector_x
            msg.linear.y=self._vector_y
            self._detect_send.send(msg)

if __name__ == '__main__':
    try:
         oDetect = ObstacleDetect('r0_o_detect')
         oDetect.run()
    except rospy.ROSInterruptException:
         pass
