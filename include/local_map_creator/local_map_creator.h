#ifndef LOCAL_MAP_CREATOR_H
#define LOCAL_MAP_CREATOR_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid"

struct laser_data{
    double angle;
    double range;
}

class Local_Map_Creator
{
 public:
     Local_Map_Creator();

 private:
    //method
    void point_cloud_callback(const sensor_msgs::LaserScan::ConstPtr&);
    void update_grid_map();//Local Mapの作成及び更新を行う

    //parameter
    int gmap_width;
    int gmap_height;
    double gmap_resolution;
    int obstacle_value;
    int safe_value;

    //member
    ros::Publisher pub_laser;
    ros::Subscriber sub_grid;
    sensor_msgs::LaserScan
    nav_msgs::OccupancyGrid
};
