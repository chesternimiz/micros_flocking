#!/usr/bin/env python

__author__ = "Zhongxuan Cai"

import sys
sys.path.append("..")
from alliance.function_unit import FunctionUnit
import actionlib
import swarm.msg
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import thread

class FGORNav(FunctionUnit):
    def __init__(self, node_name):
        FunctionUnit.__init__(self, node_name)
        #self._robot=robot_name
        self._obstacle_x = 0.0
        self._obstacle_y = 0.0
        self._max_speed = 1
        self._min_speed = -1
        self._noise_x =0.0
        self._noise_y =0.0
        self._formation_x =0.0
        self._formation_y =0.0
        self._robot_vx =0.0#already caclulated velocity vector
        self._robot_vy =0.0
        self._x = 0.0
        self._y = 0.0

    def run(self):
        FunctionUnit.init_node(self)
        test = swarm.msg.Broadcast()
        obstacle_receive = FunctionUnit.create_receive(self, "obstacle_detection", Twist, self.obstacle_on_received)
        noise_receive = FunctionUnit.create_receive(self, "noise", Twist, self.noise_on_received)
        formation_receive = FunctionUnit.create_receive(self, "formation_position", Twist, self.formation_on_received)
        robot_receive = FunctionUnit.create_receive(self, "robot_detect", Twist, self.robot_on_received)
        self._cmd_send = FunctionUnit.create_send(self, 'cmd_vel', Twist)
        position_receive = FunctionUnit.create_receive(self, "base_pose_ground_truth", Odometry, self.position_on_received)
        thread.start_new_thread(self.start,())
        FunctionUnit.spin(self)

    def obstacle_on_received(self,msg):
        self._obstacle_x=msg.linear.x
        self._obstacle_y=msg.linear.y

    def noise_on_received(self,msg):
        self._noise_x = msg.linear.x
        self._noise_y = msg.linear.y

    def formation_on_received(self,msg):
        self._formation_x = msg.linear.x - self._x
        self._formation_y = msg.linear.y - self._y

    def robot_on_received(self,msg):
        self._robot_vx = msg.linear.x
        self._robot_vy = msg.linear.y

    def position_on_received(self,msg):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

    def start(self):
        while not rospy.is_shutdown():
            msg = Twist()
            dis2 = self._obstacle_x*self._obstacle_x+self._obstacle_y*self._obstacle_y
            dis2 = pow(dis2,0.5)
            if dis2 ==0:
                dis2 = 100
            if dis2 < 1:
               if dis2 <0.2:
                  msg.linear.x += self._obstacle_x/dis2*1.1
                  msg.linear.y += self._obstacle_y/dis2*1.1
               else:
                  scale = (1-dis2)/0.8
                  msg.linear.x += self._obstacle_x/dis2*1.1*scale
                  msg.linear.y += self._obstacle_y/dis2*1.1*scale
            dis_formation = self._formation_x*self._formation_x+self._formation_y*self._formation_y
            scale = pow(dis_formation,0.5)
            div = 1
            if scale > 1:
               div = scale / 1
            msg.linear.x += self._formation_x / div
            msg.linear.y += self._formation_y / div
            #msg.linear.x += self._noise_x
            #msg.linear.y += self._noise_y
            msg.linear.x += self._robot_vx
            msg.linear.y += self._robot_vy
            speed = msg.linear.x*msg.linear.x+msg.linear.y*msg.linear.y
            speed = pow(speed,0.5)
            scale = speed / self._max_speed
            if scale < 1:
               scale = 1
            msg.linear.x /= scale
            msg.linear.y /= scale
            self._cmd_send.send(msg)
            time.sleep(0.1)
        
if __name__ == '__main__':
    try:
         fgor = FGORNav('FGORNav_node')
         fgor.run()
    except rospy.ROSInterruptException:
         pass
