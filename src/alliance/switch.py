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
class SwitchCallback(object):
    def __init__(self,send,shared_var):
        self._send = send
        self._shared_var = shared_var
    def __call__(self, msg):
        if self._shared_var[0]:
            self._send.send(msg)

class Switch(FunctionUnit):
    def __init__(self, node_name):
        self._input_list=[]
        self._output_list=[]
        self._cb_list=[]
        self._shared_list=[]
        self._shared_list.append(False)
        self._list_count = 0
        self._motive_topic_1='activate'

        FunctionUnit.__init__(self, node_name)
        self._action_mode = False
        self._last_goal = GoalID()

    def run(self):
        FunctionUnit.init_node(self)
        receive_3 = FunctionUnit.create_receive(self, self._motive_topic_1, Bool, self.motivational_shared_var_update)
        self.set_switch()
        FunctionUnit.spin(self)
        pass

    def add_pair(self,input_topic,topic_type,output_topic):
        self._output_list.append(FunctionUnit.create_send(self, output_topic, topic_type))
        self._cb_list.append(SwitchCallback(self._output_list[self._list_count],self._shared_list))
        self._input_list.append(FunctionUnit.create_receive(self, input_topic, topic_type, self._cb_list[self._list_count]))    
        self._list_count += 1

    def set_switch(self):
        pass
        #self.add_pair('switch_0/input_0',Twist,'cmd_vel')
       

    def motivational_shared_var_update(self, msg):
        self._shared_list[0] = msg.data
        if msg.data==False:
           print "switch off"
           if self._action_mode:
              self._send_cancel.send(self._last_goal)
              print "cancel_goal"
        else:
           print "switch on"


    def add_action(self,sub_action_name,pub_action_name):
        #self._action=action_client
        self._action_mode=True
        self._recieve_action_cancel = FunctionUnit.create_receive(self, sub_action_name + '/goal', MoveBaseActionGoal, self.receive_cb_goal)
        self._send_cancel= FunctionUnit.create_send(self, pub_action_name+'/cancel', GoalID)

    def receive_cb_goal(self,msg):
        print 'goal rcv'
        #if self.motivational_shared_var == True:    
        self._last_goal.stamp.secs = msg.goal_id.stamp.secs
        self._last_goal.stamp.nsecs = msg.goal_id.stamp.nsecs
        self._last_goal.id = msg.goal_id.id

if __name__ == '__main__':
    try:
         s = Switch('switch_0')
         s.run()
    except rospy.ROSInterruptException:
         pass
