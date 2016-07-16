#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "swarm/Neighbor.h"
#include "swarm/Position.h"
#include <string>
#include <list>
#include <vector>
#include <iostream>
#include <utility>
#include <cmath>
using namespace std;

#define PI 3.1415926
#define EPSILON 0.1
#define A 5
#define B 5
const double C = abs(A-B) / sqrt(4*A*B);
#define H 0.9
#define D 3.3
#define R 4
#define C1 0.5
#define C2 0.5
class NeighborHandle
{
    public:
    ros::Subscriber sub;
    double _px,_py,_vx,_vy;
    pair<double,double> _position,_velocity;
    int _r_id; 
    
    NeighborHandle(int r_id)
    {
        ros::NodeHandle n;
        stringstream ss;
        ss<<"/robot_"<<r_id<<"/position";
        sub = n.subscribe(ss.str(), 1000, &NeighborHandle::cb,this);
        _px=0;
        _py=0;
        _vx=0;
        _vy=0;
        _position=pair<double,double>(0,0);
        _velocity=pair<double,double>(0,0);
        _r_id = r_id;
    }
    
    void cb(const swarm::Position::ConstPtr & msg)
    {
        //cout<<this->_px<<" "<<_r_id<<" "<<_vy<<endl;
        //_py=1;_px=1;
        _px=msg->px;
        _py=msg->py;
        _vx=msg->vx;
        _vy=msg->vy;
        _position.first=_px;_position.second=_py;
        _velocity.first=_vx;_velocity.second=_vy;
        //_vx=1;_vy=1;//myx=1;myy=1;
        //cout<<_r_id<<endl;
    }

    void close()
    {
        sub.shutdown();
    }
};
static list<NeighborHandle*> neighbor_list;
pair<double,double> my_position=pair<double,double>(0,0);
pair<double,double> my_velocity=pair<double,double>(0,0);
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

static void neighbor_cb(const swarm::Neighbor::ConstPtr & msg)
{
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

void my_position_cb(const swarm::Position::ConstPtr & msg)
{
    my_position.first = msg->px;
    my_position.second = msg->py;
    my_velocity.first = msg->vx;
    my_velocity.second = msg->vy;
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
        pair<double,double> q_ij = get_vector(my_position,(*i)->_position);
        pair<double,double> n_ij = segma_epsilon(q_ij);
        re.first += phi_alpha(segma_norm(q_ij))*n_ij.first;
        re.second += phi_alpha(segma_norm(q_ij))*n_ij.second;
    }
    return re;
}

double a_ij(pair<double,double> j_p)
{
    return rho(segma_norm(get_vector(my_position,j_p)) / R_alpha);
}

pair<double,double> f_d()
{
    pair<double,double> re = pair<double,double>(0,0);
    for(list<NeighborHandle*>::iterator i=neighbor_list.begin();i!=neighbor_list.end();i++)
    {
        pair<double,double> p_ij = get_vector(my_velocity,(*i)->_velocity);
        re.first += a_ij((*i)->_position) * p_ij.first;
        re.second += a_ij((*i)->_position) * p_ij.second;
    }
    return re;
}

pair<double,double> f_r()
{
    pair<double,double> re = pair<double,double>(0,0);
    static pair<double,double> q_r = pair<double,double>(0,0);
    pair<double,double> p_r = pair<double,double>(0.5,0);
    q_r.first += p_r.first * 0.1;
    re.first = -C1*get_vector(q_r,my_position).first - C2*get_vector(p_r,my_velocity).first;
    re.second = -C1*get_vector(q_r,my_position).second - C2*get_vector(p_r,my_velocity).second;
    return re;
}

int main(int argc, char** argv)
{
   ros::init(argc,argv,"turtlebot_roll_node");
   ros::NodeHandle n;
   ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
   ros::Subscriber sub = n.subscribe("neighbor", 1000, neighbor_cb);
   ros::Subscriber sub_pos = n.subscribe("position", 1000, my_position_cb);
   //neighbor_list.push_back(NeighborHandle(1));
   ros::Rate loop_rate(20);
   //ros::spin();
   geometry_msgs:: Twist msg;
   while(ros::ok())
   {
      ros::spinOnce();
      msg.linear.x = (f_g().first+f_d().first+f_r().first);//*0.05;
      msg.linear.y = (f_g().second+f_d().second+f_r().second);//*0.05;
      pub.publish(msg);
      
      loop_rate.sleep();
   }
   return 0;
}
