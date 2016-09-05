#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include "micros_flocking/Neighbor.h"
#include "micros_flocking/Position.h"
#include "nav_msgs/Odometry.h"
#include <string>
#include <list>
#include <vector>
#include <iostream>
#include <utility>
#include <cmath>
#include <ctime>
#include <fstream>
using namespace std;
#define R 12

#define perror 0  //value
#define verror 0 //rate
#define D 20
#define V 0
class OdomHandle
{
    public:
    ros::Subscriber sub,neighbor_sub;
    double _px,_py,_vx,_vy;
    pair<double,double> _position,_velocity;
    vector<int> neighbor_vector;
    int _r_id; 

    OdomHandle(int r_id)
    {
        ros::NodeHandle n;
        stringstream ss;
        ss<<"/robot_"<<r_id<<"/base_pose_ground_truth";
        sub = n.subscribe(ss.str(), 1000, &OdomHandle::cb,this);


        stringstream ss2;
        ss2<<"/robot_"<<r_id<<"/neighbor";
        neighbor_sub = n.subscribe(ss2.str(), 1000, &OdomHandle::neighborcb,this);
        _px=0;
        _py=0;
        _vx=0;
        _vy=0;
        _position=pair<double,double>(0,0);
        _velocity=pair<double,double>(0,0);
        _r_id = r_id;
        neighbor_vector = vector<int>();
    }
    
    void cb(const nav_msgs::Odometry::ConstPtr & msg)
    {
      
        _px=msg->pose.pose.position.x;
        _py=msg->pose.pose.position.y;
        _vx=msg->twist.twist.linear.x;
        _vy=msg->twist.twist.linear.y;
        _position.first=_px;_position.second=_py;
        _velocity.first=_vx;_velocity.second=_vy;

      /*  swarm::Position sendmsg;
        sendmsg.px=_px+(rand()%2001-1000)/1000.0*perror;
        sendmsg.py=_py+(rand()%2001-1000)/1000.0*perror;
        sendmsg.vx=(_vx+3)*(1+(rand()%2001-1000)/1000.0*verror)-3;
        sendmsg.vy=(_vy+3)*(1+(rand()%2001-1000)/1000.0*verror)-3;
        sendmsg.theta = tf::getYaw(msg->pose.pose.orientation);
        pub.publish(sendmsg);*/
        //_vx=1;_vy=1;//myx=1;myy=1;
        //cout<<_r_id<<endl;
    }

    void neighborcb(const micros_flocking::Neighbor::ConstPtr & msg)
    {
       neighbor_vector.clear();
       for(int i=0;i< msg->data.size();i++)
       {
           neighbor_vector.push_back(msg->data[i]);
       }
    }

    void close()
    {
        sub.shutdown();
    }
};

static vector<OdomHandle*> odom_list;
int robotnum=50;
vector<vector<int> > adj_list;

double dist(int i,int j)
{
    double re=pow(odom_list[i]->_px-odom_list[j]->_px,2)+pow(odom_list[i]->_py-odom_list[j]->_py,2);
    //if(i==4)
    //cout<<re<<endl;
    return sqrt(re);
    
}

vector<double> calculate_v(vector<double> v)
{
    vector<double> re=vector<double>();
    re.push_back(0);//avg
    re.push_back(0);//std
    re.push_back(0);//max
    if(v.size()==0)
    {
         return re;
    }
    for(int i = 0 ;i< v.size();i++)
    {
         re[0]+= v[i];
         if(v[i]>re[2])
             re[2]=v[i];
    }
    re[0]/=v.size();
    for(int i=0;i<v.size();i++)
    {
         re[1]+=(v[i]-re[0])*(v[i]-re[0]);
    }
    if(v.size()==1)
       re[1]=0;
    else
       re[1]/=(v.size()-1);
    return re;
}

int main(int argc, char** argv)
{
   ros::init(argc,argv,"sim_manager");
   ros::NodeHandle n;
   srand(time(0));
   bool param_ok = ros::param::get ("~robotnum", robotnum);
   double start_time = ros::Time::now().toSec();
   for(int i=0;i<robotnum;i++)
   {
      OdomHandle *p=new OdomHandle(i);
      odom_list.push_back(p);
      adj_list.push_back(vector<int>());
   }
   //neighbor_list.push_back(NeighborHandle(1));
   ros::Rate loop_rate(20);
   ofstream fout1("/home/czx/distance.txt");
   ofstream fout2("/home/czx/velocity.txt");
   while(ros::ok())
   {
      ros::spinOnce();
      vector<double> d_error=vector<double>();
      vector<double> v_error=vector<double>();
      for(int i=0;i<robotnum;i++)
      {
          v_error.push_back(abs( sqrt(odom_list[i]->_vx*odom_list[i]->_vx+odom_list[i]->_vy*odom_list[i]->_vy) - V) );
          for(int j=0;j<odom_list[i]->neighbor_vector.size();j++)
          {
              d_error.push_back(abs(dist(i,odom_list[i]->neighbor_vector[j]) - D));
          }
      }
      vector<double> d_cal = calculate_v(d_error);
      vector<double> v_cal = calculate_v(v_error);
      double time_now = ros::Time::now().toSec() - start_time;
      cout<<time_now<<endl;
      cout<<"distance "<<d_cal[0]<<' '<<d_cal[1]<<' '<<d_cal[2]<<endl;
      cout<<"velocity "<<v_cal[0]<<' '<<v_cal[1]<<' '<<v_cal[2]<<endl;
      fout1<<time_now<<' '<<d_cal[0]<<' '<<d_cal[1]<<' '<<d_cal[2]<<endl;
      fout2<<time_now<<' '<<v_cal[0]<<' '<<v_cal[1]<<' '<<v_cal[2]<<endl;
      loop_rate.sleep();
   }
   return 0;
}
