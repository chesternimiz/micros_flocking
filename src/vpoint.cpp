#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include "micros_flocking/Neighbor.h"
#include "micros_flocking/Position.h"
#include "micros_flocking/Gradient.h"
#include "nav_msgs/Odometry.h"
#include <string>
#include <list>
#include <vector>
#include <iostream>
#include <utility>
#include <cmath>
#include <ctime>
#include <boost/thread/thread.hpp> 
using namespace std;

double PI=acos(-1);
#define EPSILON 0.1
#define A 5
#define B 5
const double C = abs(A-B) / sqrt(4*A*B);
#define H 0.2
#define D 20
#define R 25
#define C1 0.05
#define C2 0.3
#define rspeedlimit 1
double basespeed = 1;
#define ploss 0 //max1000
#define diffdrive true
#define max_turn 0.7
bool move_vl = false;
bool neighbor_loss=false;
int hz=10;
bool delay_enabled = false;
int delay_time = 200;

#define krho 3
double kalpha =8;
#define kbeta -1.5


double   interval=1.0/hz;
double pm1=0.3,pm2=1,pm3=0.3;
class NeighborHandle
{
    public:
    ros::Subscriber sub;
    double _px,_py,_vx,_vy;
    pair<double,double> _position,_velocity;
    int _r_id; 
    double mypm_g;
    int gradient;
    NeighborHandle(int r_id)
    {
        ros::NodeHandle n;
        stringstream ss;
        ss<<"/robot_"<<r_id<<"/vpoint_position";
        sub = n.subscribe(ss.str(), 1000, &NeighborHandle::cb,this);
        _px=0;
        _py=0;
        _vx=0;
        _vy=0;
        _position=pair<double,double>(0,0);
        _velocity=pair<double,double>(0,0);
        _r_id = r_id;
        mypm_g=1;
        gradient = -1;
    }
    
    void cb(const micros_flocking::Position::ConstPtr & msg)
    {
        //cout<<this->_px<<" "<<_r_id<<" "<<_vy<<endl;
        //_py=1;_px=1;
        if(rand()%1000<ploss)
             return;
        
        _px=msg->px;
        _py=msg->py;
        _vx=msg->vx;
        _vy=msg->vy;
        _position.first=_px;_position.second=_py;
        _velocity.first=_vx;_velocity.second=_vy;
        gradient=msg->gradient;
        //cout<<"neighbor pose updated"<<endl;
        //_vx=1;_vy=1;//myx=1;myy=1;
        //cout<<_r_id<<endl;
    }

    void close()
    {
        sub.shutdown();
    }
};
static list<NeighborHandle*> neighbor_list;
pair<double,double> my_vpoint_position=pair<double,double>(0,0);
pair<double,double> my_vpoint_velocity=pair<double,double>(0,0);
pair<double,double> my_position=pair<double,double>(0,0);
pair<double,double> my_velocity=pair<double,double>(0,0);
double my_theta = 0;
bool findInMyList(int r_id)
{
    for(list<NeighborHandle*>::iterator i=neighbor_list.begin();i!=neighbor_list.end();i++)
    {
        if (r_id == (*i)->_r_id)
            return true;
    }
    return false;
}

bool findInVector(int r_id,vector<int> v)
{
    for(int i=0;i<v.size();i++)
        if(r_id == v[i])
            return true;
    return false;
}

static void neighbor_cb(const micros_flocking::Neighbor::ConstPtr & msg)
{
    if(rand()%1000<ploss && neighbor_loss)
             return;
    for(int i=0;i< msg->data.size();i++)
        if (!findInMyList(msg->data[i]))
        {
            //neighbor_list.push_back(NeighborHandle(msg->data[i]));
            NeighborHandle* x= new NeighborHandle(msg->data[i]);
            neighbor_list.push_back(x);
            //cout<<"add"<<endl;
        }
    for(list<NeighborHandle*>::iterator i=neighbor_list.begin();i!=neighbor_list.end();i++)
    {
        if(!findInVector((*i)->_r_id,msg->data))
        {
            (*i)->close();
            delete (*i);
            i=neighbor_list.erase(i);
            //cout<<"close"<<endl;
        }
    }
    //cout<<"exit callback"<<endl;
    
}

double my_gradient = -1;
void my_vpoint_position_cb(const micros_flocking::Position::ConstPtr & msg)
{
    my_vpoint_position.first = msg->px;
    my_vpoint_position.second = msg->py;
    my_vpoint_velocity.first = msg->vx;
    my_vpoint_velocity.second = msg->vy;
    //my_theta = msg->theta;
    my_gradient = msg->gradient;
    //cout<<"my pose updated"<<endl;
}

void my_position_cb(const micros_flocking::Position::ConstPtr & msg)
{
    my_position.first = msg->px;
    my_position.second = msg->py;
    my_velocity.first = msg->vx;
    my_velocity.second = msg->vy;
    my_theta = msg->theta;

    
    //my_gradient = msg->gradient;
    //cout<<"my pose updated"<<endl;
}

pair<double,double> get_vector(pair<double,double> start,pair<double,double> end)
{
    pair<double,double> re=pair<double,double>(0,0);
    re.first=end.first-start.first;
    re.second=end.second-start.second;
    return re;
}

double segma_norm(pair<double,double> v)
{
    double re = EPSILON*(v.first*v.first+v.second*v.second);
    re = sqrt(1+re)-1;
    re /= EPSILON;
    return re;
}

double R_alpha = segma_norm(pair<double,double>(R,0));
double D_alpha = segma_norm(pair<double,double>(D,0));

pair<double,double> segma_epsilon(pair<double,double> v)
{
    pair<double,double> re = pair<double,double>(0,0);
    double scale = 1+EPSILON*(v.first*v.first+v.second*v.second);
    scale = sqrt(scale);
    re.first = v.first / scale;
    re.second = v.second / scale;
    return re;
}

double segma_1(double z)
{
    return z / sqrt(1+z*z);
}

double phi(double z)
{
    return 0.5*((A+B)*segma_1(z+C)+A-B);
}

double rho(double z)
{
    if(z<H)
        return 1;
    if(z>1)
        return 0;
    return 0.5*(1+cos(PI*(z-H)/(1-H)));
}

double phi_alpha(double z)
{
    return rho(z/R_alpha)*phi(z-D_alpha);
}

pair<double,double> f_g()
{
    pair<double,double> re = pair<double,double>(0,0);
    for(list<NeighborHandle*>::iterator i=neighbor_list.begin();i!=neighbor_list.end();i++)
    {
        pair<double,double> q_ij = get_vector(my_vpoint_position,(*i)->_position);
        pair<double,double> n_ij = segma_epsilon(q_ij);
        re.first += phi_alpha(segma_norm(q_ij))*n_ij.first*(*i)->mypm_g;
        re.second += phi_alpha(segma_norm(q_ij))*n_ij.second*(*i)->mypm_g;
    }
    return re;
}

double a_ij(pair<double,double> j_p)
{
    return rho(segma_norm(get_vector(my_vpoint_position,j_p)) / R_alpha);
}

pair<double,double> f_d()
{
    pair<double,double> re = pair<double,double>(0,0);
   // int count=0;
    for(list<NeighborHandle*>::iterator i=neighbor_list.begin();i!=neighbor_list.end();i++)
    {
        pair<double,double> p_ij = get_vector(my_vpoint_velocity,(*i)->_velocity);
        re.first += a_ij((*i)->_position) * p_ij.first;
        re.second += a_ij((*i)->_position) * p_ij.second;
       // count++;
    }
    //cout<<count<<endl;
    return re;
}

pair<double,double> q_r = pair<double,double>(0,0);
pair<double,double> p_r = pair<double,double>(basespeed,basespeed);
pair<double,double> f_r()
{
    pair<double,double> re = pair<double,double>(0,0);
    
    
    q_r.first += p_r.first * interval;
    q_r.second += p_r.second * interval;
    //cout<<q_r.first<<' '<<q_r.second;
    re.first = -C1*get_vector(q_r,my_vpoint_position).first - C2*get_vector(p_r,my_vpoint_velocity).first;
    re.second = -C1*get_vector(q_r,my_vpoint_position).second - C2*get_vector(p_r,my_vpoint_velocity).second;
    //cout<<q_r.first<<' '<<q_r.second<<' '<<get_vector(q_r,my_vpoint_position).first<<' '<<get_vector(q_r,my_vpoint_position).second<<endl;
    return re;
}



void spin_thread()
{
    while(ros::ok())
    {
       if(delay_enabled)
        {
           int sleep_sec = delay_time;
            boost::this_thread::sleep(boost::posix_time::millisec(sleep_sec));
        }
       else
       {
             ros::Rate loop_rate(100);
             loop_rate.sleep();
        }
       ros::spinOnce();
    }
}

    double _px,_py,_vx,_vy;
    void p_cb(const nav_msgs::Odometry::ConstPtr & msg)
    {
        
        _px=msg->pose.pose.position.x;
        _py=msg->pose.pose.position.y;
        _vx=msg->twist.twist.linear.x;
        _vy=msg->twist.twist.linear.y;
    }

int main(int argc, char** argv)
{
   srand(time(0));
   ros::init(argc,argv,"turtlebot_roll_node");
   ros::NodeHandle n;
   int vpointid;
   bool param_ok = ros::param::get ("vpointid", vpointid);
   if(!param_ok)cout<<"param error"<<endl;
   ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
   stringstream ss;
   ss<<"/robot_"<<vpointid<<"/vpoint_position";
   ros::Subscriber sub_vpoint_pos = n.subscribe(ss.str(), 1000, my_vpoint_position_cb);
   ros::Subscriber sub_my_pos = n.subscribe("base_pose_ground_truth", 1000, p_cb);
  // cout<<vpointid<<endl;
   ros::Rate loop_rate(hz);
 
   geometry_msgs:: Twist msg;
   geometry_msgs:: Twist sendmsg;

   boost::thread thrd(&spin_thread);

   while(_px==0.0 || my_vpoint_position.first==0.0){loop_rate.sleep();}
   double kkk=8;
   double rk=1.0;
   while(ros::ok())
   {
double delta_x = my_vpoint_position.first - _px;
double delta_y = my_vpoint_position.second - _py;
double dist = sqrt(delta_x*delta_x+delta_y*delta_y);
      cout<<dist<<endl;
      if(dist<1.0) kkk=3;
      else if(dist<5.0) kkk=5*(dist-1.0)/4+3;
      else kkk=10;
      
      sendmsg.linear.x = rk*delta_x+my_vpoint_velocity.first;
      sendmsg.linear.y = rk*delta_y+my_vpoint_velocity.second;
      if(dist<0.5) rk=dist;
      //sendmsg.linear.x = kkk*delta_x;
      //sendmsg.linear.y = kkk*delta_y;
      //sendmsg.linear.x = my_vpoint_velocity.first;
      //sendmsg.linear.y = my_vpoint_velocity.second;
      pub.publish(sendmsg);
      loop_rate.sleep();
   }
   return 0;
}
