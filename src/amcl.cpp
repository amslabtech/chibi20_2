#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"

#include <isostream>
#include <random>
#include <math.h>


class Particle
{
public:
    Particle();
    void p_init(double,double,double);
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
    float x_cov;
    float y_cov;
    float yaw_cov;

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

//乱数生成(0~1)
float random()
{
    static int flag;

    if(flag == 0){
        srand((unsigned int)time(NULL));
        flag = 1;
    }

    return ((float)rand() + 1.0)/((float)RAND_MAX + 2.0);
}

//Box-Muller法
float random_normal(float mu,float sigma)
{
    float z = sqrt(-2.0*log(random())*sin(2.0*M_PI*random()));

    return (mu + sigma*z);
}

Particle::Particle()
{
    pose.header.frame_id = "map";

    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    quaternionTFToMsg(tf::createQuaternionFromYaw(0),pose.pose.orientation)    ;

    weight = 1 / (double)N;
}

void Particle::p_set(double x,double y,double yaw)
{
    pose.pose.position.x = random_normal(x,x_cov);
    pose.pose.position.y = random_normal(y,y_cov);
    quaternionTFToMsg(tf::createQuaternionFromYaw(random_normal(yaw,yaw_cov),pose.pose.orientation);
}

void Particle::p_motion_updata(geometry_msgs::PoseStamped current,geometry_msgs)
{
    float dx;
    float dy;
    float dyaw;
    float dist;

    dx
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
