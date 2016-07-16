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
import math
import random

class NeiPosCallback(object):
    def __init__(self,index_id,pos_list,vel_list):
        self._id=index_id
        self._pos_list=pos_list
        self._vel_list=vel_list
        
    def __call__(self, msg):
        self._pos_list[self._id*2]=msg.x
        self._pos_list[self._id*2+1]=msg.y
        self._vel_list[self._id*2]=msg.vx
        self._vel_list[self._id*2+1]=msg.vy
        
class Alpha(FunctionUnit):

    def __init__(self, node_name):
        FunctionUnit.__init__(self, node_name)
        self._sub_list=[]
        self._neighbor_position=[]
        self._neighbor_vel=[]
        self._neighbor_list=[]
        self._cb_list=[]
        self._position=[0,0]
        self._velocity=[0,0]
        #self._pub_list=[]
        
        self._epsilon = 0.1 #segma_norm param
        self._a = 5
        self._b = 5
        self._c = abs(self._a - self._b)/ pow(4*self._a*self._b,0.5)
        self._h = 0.2 #param: rho function for alpha agent
        self._d = 7
        self._r = 1.2*self._d
        
        self._r_alpha = self.segma_norm2([self._r,0])
        self._d_alpha = self.segma_norm2([self._d,0])
        self._c1=0.5
        self._c2=0.5

    def run(self):
        FunctionUnit.init_node(self)
        self._send = FunctionUnit.create_send(self, 'cmd_vel', Twist)
        self._mypos_rcv=FunctionUnit.create_receive(self, "position", swarm.msg.Position, self.position_on_received)
        self._rcv=FunctionUnit.create_receive(self, "neighbor", swarm.msg.Neighbor, self.neighbor_on_received)
        sendmsg=Twist()
        sendmsg.linear.x=random.uniform(-1,1)
        sendmsg.linear.y=random.uniform(-1,1)
        while not rospy.is_shutdown():
            time.sleep(0.1)
            u_0 = self.f_g()[0]+self.f_d()[0] #+self.f_r()[0]
            u_1 = self.f_g()[1]+self.f_d()[1] #+self.f_r()[1]
            sendmsg.linear.x+=u_0
            sendmsg.linear.y+=u_1
            #self._send.send(sendmsg)
        FunctionUnit.spin(self)

    def position_on_received(self,msg):
        self._position[0]=msg.x
        self._position[1]=msg.y
        self._velocity[0]=msg.vx
        self._velocity[1]=msg.vy
        
    def neighbor_on_received(self,msg):
        '''
        for i in range(0,len(self._neighbor_list)):
            if i == len(self._neighbor_list):
                break
            if not self.find(self._neighbor_list[i],msg.data):
                self._sub_list[i].unregister()
                del self._neighbor_list[i]
                del self._sub_list[i]
                del self._cb_list[i]
                for j in range(i,len(self._cb_list)):
                    self._cb_list[j]._id -= 1
                del self._neighbor_position[2*i+1]
                del self._neighbor_position[2*i]
                del self._neighbor_vel[2*i+1]
                del self._neighbor_vel[2*i]
        '''
        for i in range(0,len(msg.data)):
            if not self.find(msg.data[i],self._neighbor_list):
                self._neighbor_position.append(0)
                self._neighbor_position.append(0)
                self._neighbor_vel.append(0)
                self._neighbor_vel.append(0)
                self._neighbor_list.append(msg.data[i])
                self._cb_list.append(NeiPosCallback(len(self._neighbor_list)-1,self._neighbor_position,self._neighbor_vel))
                self._sub_list.append(FunctionUnit.create_receive(self, "/robot_"+str(msg.data[i])+"/position", swarm.msg.Position, self._cb_list[len(self._cb_list)-1]))
        '''
        self._neighbor_list=[]
        self._sub_list=[]
        self._cb_list=[]
        self._neighbor_position=[]
        self._neighbor_vel=[]
        for i in range(0,len(msg.data)):
            self._neighbor_position.append(0)
            self._neighbor_position.append(0)
            self._neighbor_vel.append(0)
            self._neighbor_vel.append(0)
            self._neighbor_list.append(msg.data[i])
            self._cb_list.append(NeiPosCallback(i,self._neighbor_position,self._neighbor_vel))
            self._sub_list.append(FunctionUnit.create_receive(self, "/robot_"+str(msg.data[i])+"/position", swarm.msg.Position, self._cb_list[i]))
        '''    
    def find(self,r_id,n_list):
        for i in range(0,len(n_list)):
            if r_id == n_list[i]:
                return True
        return False
                
    def get_position(self,r_id): #index id
        re = []
        re.append(self._neighbor_position[r_id*2])
        re.append(self._neighbor_position[r_id*2+1])
        return re
    
    def get_velocity(self,r_id): #index id
        re = []
        re.append(self._neighbor_vel[r_id*2])
        re.append(self._neighbor_vel[r_id*2+1])
        return re
    
    def get_vector(self,start,end):
        re =[]
        re.append(end[0]-start[0])
        re.append(end[1]-start[1])
        return re
    
    def segma_norm2(self,vector2):
        scale = pow(vector2[0]*vector2[0]+vector2[1]*vector2[1],0.5)
        scale = 1 + self._epsilon * scale
        scale = pow(scale,0.5)
        scale = scale -1
        scale = scale / self._epsilon
        return scale
        
    def segma_norm2_gradient(self,vector2):
        scale = pow(vector2[0]*vector2[0]+vector2[1]*vector2[1],0.5)
        scale = 1 + self._epsilon * scale
        scale = pow(scale,0.5)
        re = []
        re.append(vector2[0]/scale)
        re.append(vector2[1]/scale)
        return re
        
    def segma1(self,z): #used in phi
        return z / pow(1+z*z,0.5)
        
    def phi(self,z): #used in phi_alpha
        return 0.5*((self._a+self._b)*self.segma1(z+self._c)+self._a-self._b)
        
    def rho(self,z): #used in phi_alpha ,a_ij
        if z < self._h:
            return 1
        if z>1:
            return 0
        return 0.5*(1 + math.cos(math.pi*(z-self._h)/(1-self._h)))
        
    def phi_alpha(self,z):
        return self.rho(z/self._r_alpha)*self.phi(z-self._d_alpha)
        
    def f_g(self):
        re = []
        re.append(0)
        re.append(0)
        for j in range (0,len(self._neighbor_position)/2):
            if j == len(self._neighbor_position)/2:
                break
            q_ij = self.get_vector(self._position, self.get_position(j))
            n_ij = self.segma_norm2_gradient(q_ij)
            re[0]+=self.phi_alpha(self.segma_norm2(q_ij))*n_ij[0]
            re[1]+=self.phi_alpha(self.segma_norm2(q_ij))*n_ij[1]
        return re
        
    def a_ij(self,j):
        return self.rho(self.segma_norm2(self.get_vector(self._position, self.get_position(j))) / self._r_alpha) 
        
    def f_d(self):
        re = [0,0]
        for j in range (0,len(self._neighbor_position)/2):
            if j == len(self._neighbor_position)/2:
                break
            p_ij = self.get_vector(self._velocity, self._neighbor_vel)
            re[0] += self.a_ij(j) * p_ij[0]
            re[1] += self.a_ij(j) * p_ij[1]
        return re
        
    def f_r(self):
        re = [0,0]
        q_r = [20,20]
        p_r = [0,0]
        re[0] = - self._c1 * self.get_vector(q_r,self._position)[0] - self._c2 * self.get_vector(p_r,self._velocity)[0]
        re[1] = - self._c1 * self.get_vector(q_r,self._position)[1] - self._c2 * self.get_vector(p_r,self._velocity)[1]
        return re
        


if __name__ == '__main__':
    try:
         rb = Alpha('alpha')
         rb.run()
    except rospy.ROSInterruptException:
         pass
