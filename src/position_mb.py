#!/usr/bin/env python

__author__ = 'Zhongxuan Cai'
from alliance.motivational_behavior import MotivationalBehavior
from alliance.switch import Switch
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import swarm.msg
import rospy
from middle_abstraction.function_unit import FunctionUnit
class PositionMotivationalBehavior(MotivationalBehavior):
    def __init__(self, node_name,  switch_topic, filename = None,heartbeat_topic = None):
        MotivationalBehavior.__init__(self, node_name,  switch_topic, filename = None,heartbeat_topic = None)
        
    def user(self):#user defined inc(fast)
        self._fast_receive = FunctionUnit.create_receive(self, 'inc_input', Twist, self.fast_on_received)
        self._fast = 0

    def fast_on_received(self,msg):
        dist = pow(msg.linear.x*msg.linear.x+msg.linear.y*msg.linear.y,0.5)
        if dist == 0:
            self._fast = 110
        else: 
            if dist > 3:
                self._fast = 10
            else:
                self._fast = (3-dist)/5*100+10

if __name__ == '__main__':
    try:
         pmb = PositionMotivationalBehavior('bs1','activate')
         pmb.set_sensory_feedback(1)
         #pmb.enable_udf_inc()
         pmb.run()
    except rospy.ROSInterruptException:
         pass
