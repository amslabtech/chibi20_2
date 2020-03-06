#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"

#include <isostream>
#include <random>
#include <math.h>

//乱数生成
std::random_device rnd;
std::mt19937 mt(rnd());
std::uniform_real_distribution <> dist(0.0,1.0);

class Particle
{
public:
    Particle();
    void init(double,double,double);
    void motion();
    void measurement();

    void map_callback(const nav_msgs::OccupancyGrid::Co    nstPtr&);

private:
    //method
    void odometry_callback(const nav_msgs::Odometry::ConstPtr&);
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr&);

    double rand_normal();
    double rand_normal_mu_sigma();

    //parameter
    double weight;

    //member
    nav_msgs::OccupancyGrid grid;
    nav_msgs::Odometry odometry;
    sensor_msgs::LaserScan laser;
}

void Particle::odometry_callback(const nav_msgs::Odometry::ConstPtr& odo)
{
    odometry = *odo;
}

void Particle::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    grid
}


void Particle::laser_callback(const sensor_msgs::LaserScan::ConstPtr& lsr)
{
    laser = *lsr;
}

//Box-Muller法(1)
double rand_normal()
{
    return sqrt(-2.0 * log(rand())) * cos(2.0*M_PI*rand());
}

//Box-Muller法(2)
double rand_normal_mu_sigma(double mu,double sigma)
{
    return mu + sigma * rand_normal();
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"amcl");
    ros::NodeHandle nh;
    ros::NodeHandle amcl_nh_("~");

    //Subscriber
    ros::Subscriber sub_map      = nh.subscribe("/map",1,&Particle::map_callback,this);
    ros::Subscriber sub_laser    = nh.subscribe("/scan",1,&Particle::lasre_callback,this);
    ros::Subscriber sub_odometry = nh.subscribe("/roomba/odometry",1,&Particle::odometry_callback.this);

    //Publisher

    return 0;
}
