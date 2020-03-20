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
<<<<<<< HEAD
     Grid_Map_Creator();
     void process();
=======
     Local_Map_Creator();
>>>>>>> 3d56d444940b8450885e8c15f872baab03ac3b3b

 private:
    //method
    void point_cloud_callback(const sensor_msgs::LaserScan::ConstPtr&);
<<<<<<< HEAD
    void make_grid_map();


    //parameter
    std::vector<std::vector<int>> grid_map;
    int row;
    int column;

=======
    void update_grid_map();//Local Mapの作成及び更新を行う

    //parameter
    int gmap_width;
    int gmap_height;
    double gmap_resolution;
    int obstacle_value;
    int safe_value;
>>>>>>> 3d56d444940b8450885e8c15f872baab03ac3b3b

    //member
    ros::Publisher pub_laser;
    ros::Subscriber sub_grid;
    sensor_msgs::LaserScan
    nav_msgs::OccupancyGrid
};
