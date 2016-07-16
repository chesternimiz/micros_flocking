#!/usr/bin/env python

__author__ = 'Zhongxuan Cai'
from alliance.motivational_behavior import MotivationalBehavior
from alliance.switch import Switch
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

ob7 = Switch('robot_0_switch_0','robot_0/switch_0/input_0',Twist,'robot_0/cmd_vel')#nodename,subtopic,type,pubtopic
ob7.start()
