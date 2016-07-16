#!/usr/bin/env python
# coding=utf-8

__author__ = "Zhongxuan Cai"

import sys
sys.path.append("..")
from middle_abstraction.function_unit import FunctionUnit
from std_msgs.msg import Bool
from multi_robot_patrol.msg import Heartbeat
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionGoal
import rospy
from geometry_msgs.msg import Twist
from alliance.switch import Switch

class PositionSwitch(Switch):
    def __init__(self, node_name):
        Switch.__init__(self,node_name)
    def set_switch(self):
        self.add_pair('formation_position',Twist,'output')

if __name__ == '__main__':
    try:
         ps = PositionSwitch('switch_0')
         ps.run()
    except rospy.ROSInterruptException:
         pass
