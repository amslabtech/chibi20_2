#ifndef _AMCL_H
#define _AMCL_H

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
std::uniform_real_distribution <> rand1(0.0,1.0);

class Particle
{
public:
    Particle();

    void init(double,double,double);
    void motion();
    void measurement();

    double weight;

private:
    //method
    ////parameter
}

void map_callback()
{
}

void laser_callback()
{
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
