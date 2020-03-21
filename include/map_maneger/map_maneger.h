#ifndef MAP_MANEGER_H
#define MAP_MANEGER_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

class Map_Maneger
{
public:
    Map_Maneger();
    void process();

private:

    struct Coordinate
    {
        int x;
        int y;
    };
    //method
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr&);
    void blocked_map_callback(const nav_msgs::OccupancyGrid::ConstPtr&);
    void fix_map();
    void set_grid_map_parametor();
    void move_map_parallel();
    void turn_map();


    //parameter
    int row;
    int column;
    int hz;
    Coordinate adjust;
    Coordinate move_length;
    double degree;
    //member
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Subscriber sub_map;
    ros::Subscriber sub_blocked_map;
    ros::Publisher pub_fixed_map;
    nav_msgs::OccupancyGrid original_map;
    nav_msgs::OccupancyGrid fixed_map;
    std::vector<std::vector<int>> grid_map;


};

#endif//MAP_MANEGER_H

