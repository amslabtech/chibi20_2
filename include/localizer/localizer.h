#ifndef _LOCALIZER_H
#define _LOCALIZER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"

#include <random>
#include <math.h>

class Particle
{
public:
    Particle();

private:
    //method
    void init();
    void motion();
    void measurement();

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& lsr);
    void odometry_callback(const nav_msgs::Odometry::ConstPtr& odo);

    //parameter
    int N;          //Particle
    double weight;  //Likelhood

    //member
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::Subscriber sub_map;
    ros::Subscriber sub_laser;
    ros::Subscriber sub_odometry;

    ros::Publisher pub_GPP;
    ros::Publisher pub_LMC;
};

#endif  //LOCALIZER_H
