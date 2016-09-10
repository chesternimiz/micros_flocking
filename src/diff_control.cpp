#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include "micros_flocking/Neighbor.h"
#include "micros_flocking/Position.h"
#include "micros_flocking/Gradient.h"
#include <string>
#include <list>
#include <vector>
#include <iostream>
#include <utility>
#include <cmath>
#include <ctime>
#include <boost/thread/thread.hpp> 
using namespace std;

#define krho 0.3
double kalpha =0.8;
#define kbeta -0.15
double hz = 10;
double PI=acos(-1);
pair<double,double> target_position = pair<double,double>(0,0);
pair<double,double> my_position=pair<double,double>(0.1,0.1);
pair<double,double> my_velocity=pair<double,double>(0,0);
double my_theta = 0;
double my_gradient = 0;

void my_position_cb(const micros_flocking::Position::ConstPtr & msg)
{
    my_position.first = msg->px;
    my_position.second = msg->py;
    my_velocity.first = msg->vx;
    my_velocity.second = msg->vy;
    my_theta = msg->theta;
    my_gradient = msg->gradient;
    //cout<<"my pose updated"<<endl;
}

int main(int argc, char** argv)
{
   srand(time(0));
   ros::init(argc,argv,"diff_control");
   ros::NodeHandle n;
   ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
   ros::Subscriber sub_pos = n.subscribe("position", 1000, my_position_cb);
   
   ros::Rate loop_rate(hz);
   while(ros::ok())
   {
      ros::spinOnce();
      double delta_x = -target_position.first +my_position.first;
      double delta_y = -target_position.second +my_position.second;
      double alpha = atan(delta_y/delta_x) - my_theta;
      double beta = - alpha - my_theta;
/*
      double beta = PI/2.0;
      //if(delta_x ==0)
         //if(delta_y < 0)
            //beta = -PI/2.0;
      //else{
         beta = atan(delta_y/delta_x);
         cout<<"updated"<<endl;
         if(delta_x < 0)
            if(delta_y >= 0)
               beta += PI;
            else
               beta -=PI;
      //}
      cout<<"firstbeta "<<beta<<' '<<delta_x<<' '<<delta_y<<endl;
      beta = - beta;
      double alpha = - beta - my_theta;
*/
      double rho = sqrt(delta_x*delta_x+delta_y*delta_y);
      geometry_msgs:: Twist sendmsg;
      sendmsg.linear.x = krho*rho;
      sendmsg.angular.z = kalpha*alpha + kbeta*beta;
      cout<<alpha<<' '<<beta<<' '<<my_theta<<endl;
      pub.publish(sendmsg);
      loop_rate.sleep();
   }
   
}
