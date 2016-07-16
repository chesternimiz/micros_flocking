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
class MultiSwitchCallback(object):
    def __init__(self,send,shared_var,num,bs_id):
        self._send = send
        self._shared_var = shared_var
        self._num = num
        self._bs_id = bs_id
    def __call__(self, msg):
        for i in range (0,self._num):
            if self._shared_var[self._bs_id][i]:
                self._send[i].send(msg)

class MultiSwitch(FunctionUnit):
    def __init__(self, node_name):
        self._input_list=[]
        self._output_list=[]
        self._cb_list=[]
        self._shared_list=[]

        FunctionUnit.__init__(self, node_name)
        self._action_mode = False
        self._last_goal = GoalID()

    def run(self):
        FunctionUnit.init_node(self)
        self._num = rospy.get_param('~num',1)
        for i in range (0,self._num):#bs_id
            self._shared_list.append([])
            for j in range (0,self._num):#r_id
                self._shared_list[i].append(False)
        receive_3 = FunctionUnit.create_receive(self, '/heart_beat', Bool, self.motivational_shared_var_update)
        self.set_type(Twist)
        for i in range (0,self._num):
            self._output_list.append(FunctionUnit.create_send(self, 'output_'+str(i), self._type))
        for i in range (0,self._num):
            self._cb_list.append(MultiSwitchCallback(self._output_list,self._shared_list,self._num,i))
            self._input_list.append(FunctionUnit.create_receive(self, 'input_'+str(i), self._type, self._cb_list[i]))
        FunctionUnit.spin(self)
        pass

    def set_type(self,topic_type):
        self._type = topic_type
        pass
        #self.add_pair('switch_0/input_0',Twist,'cmd_vel')
       

    def motivational_shared_var_update(self, msg):
        robot_id = msg.robot_ID
        bs_id = msg.behavior_set_ID
        if msg.heartbeat == True:
            for i in range (0,self._num):
                if i == robot_id:
                    self._shared_list[bs_id][i] = True
                else:
                    self._shared_list[bs_id][i] = False
        else:
            self._shared_list[bs_id][robot_id] = False


if __name__ == '__main__':
    try:
         ms = MultiSwitch('switch_0')
         ms.run()
    except rospy.ROSInterruptException:
         pass
