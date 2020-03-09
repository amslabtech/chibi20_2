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

 private:
    //method
    void point_cloud_callback(const sensor_msgs::LaserScan::ConstPtr&);
    void pose_callback(const geometry_msgs::Pose2D::ConstPtr&);
    void get_global_map();
    void update_grid_map();//Local Mapの作成及び更新を行う
    void compare_maps();
    void update_global_map();


    //parameter

    //member
    sensor_msgs::LaserScan

};
