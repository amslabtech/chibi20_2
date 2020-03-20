#ifndef LOCAL_MAP_CREATOR_H
#define LOCAL_MAP_CREATOR_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose2D"
#include "nav_msgs/OccupancyGrid"

class Grid_Map_Creator
{
 public:
     Grid_Map_Creator();
     void process();

 private:
    //method
    void point_cloud_callback(const sensor_msgs::LaserScan::ConstPtr&);
    void make_grid_map();


    //parameter
    std::vector<std::vector<int>> grid_map;
    int row;
    int column;


    //member
    sensor_msgs::LaserScan

};
