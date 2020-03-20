#include "ros/ros.h"
#include "local_map_creator/local_map_creator.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"

#include <math.h>
#include <random>
#include <iostream>
#include <vector>

int gmap_width = 200;
int gmap_height = 200;
double gmap_resolution = 0.05; //0.05m
int obstacle_value = 100;
int safe_value = 0;

struct laser_data{
    double angle;
    double range;
}

laser_data ldata[];
int 2d_map[][];

void point_cloud__callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::cout << "recieved laser";
    polar_coordinates();
}

void polar_coordinates()
{
    sensor_msgs::LaserScan _msg = *msg;

    for(int i=0;i<1080;i++){
    ldata.angle[i] = _msg.angle_min - range_resolution * _msg.angle_increment;
    ldata.range[i] = _msg.ranges[i];
    }

}

void update_grid_map()
{
    obstacle();
}
void obstacle(laser_data[])
{
    int x = 0;
    int y = 0;
    int x_translation = gmap_width  /2;
    int y_translation = gmap_height /2;

    for(int i=0;i<1080;i++)
    {
    x = floor(ldata.range * cos(ldata.angle[i]) );
    y = floor(ldata.range * sin(ldata.angle[i]) );
    2d_map[x+x_translation][y+y_translation] = obstacle_value;
        for(double j = ldata.range;j>0;j = j-map_resolution)
        {
        x = floor(ldata.range * cos(ldata.angle[i]) );
        y = floor(ldata.range * sin(ldata.angle[i]) );
        2d_map[x+x_translation][y+y_translation] = safe_value;
        }
    }
}
