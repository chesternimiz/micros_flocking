#!/usr/bin/env python

__author__ = "Zhongxuan Cai"

import sys
sys.path.append("..")
from alliance.function_unit import FunctionUnit
import actionlib
import multi_robot_patrol.msg
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import thread

class Avoid(FunctionUnit):
    def __init__(self, node_name):
        FunctionUnit.__init__(self, node_name)
        #self._robot=robot_name
        self._obstacle_x = 0.0
        self._obstacle_y = 0.0
        self._max_speed = 1
        self._min_speed = -1
        self._noise_x =0.0
        self._noise_y =0.0

    def run(self):
        FunctionUnit.init_node(self)
        obstacle_receive = FunctionUnit.create_receive(self, "obstacle_detection", Twist, self.obstacle_on_received)
        noise_receive = FunctionUnit.create_receive(self, "noise", Twist, self.noise_on_received)
        self._cmd_send = FunctionUnit.create_send(self, 'cmd_vel', Twist)
        thread.start_new_thread(self.start,())
        FunctionUnit.spin(self)

    def obstacle_on_received(self,msg):
        self._obstacle_x=msg.linear.x
        self._obstacle_y=msg.linear.y

    def noise_on_received(self,msg):
        self._noise_x = msg.linear.x
        self._noise_y = msg.linear.y

    def start(self):
        while not rospy.is_shutdown():
            dis2 = self._obstacle_x*self._obstacle_x+self._obstacle_y*self._obstacle_y
            if dis2 ==0:
                dis2 = 100
            if dis2 < 1:
               scale = 0.1 / dis2 -0.1
               msg = Twist()
               msg.linear.x = self._obstacle_x*scale
               msg.linear.y = self._obstacle_y*scale
               msg.linear.x += self._noise_x
               msg.linear.y += self._noise_y
               if msg.linear.x > self._max_speed:
                  msg.linear.x = self._max_speed
               if msg.linear.x < self._min_speed:
                  msg.linear.x = self._min_speed
               if msg.linear.y > self._max_speed:
                  msg.linear.y = self._max_speed
               if msg.linear.y < self._min_speed:
                  msg.linear.y = self._min_speed
               self._cmd_send.send(msg)
            time.sleep(0.1)
        
if __name__ == '__main__':
    try:
         av = Avoid('avoid')
         av.run()
    except rospy.ROSInterruptException:
         pass
