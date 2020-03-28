#ifndef LOCAL_MAP_CREATOR_H
#define LOCAL_MAP_CREATOR_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

class Local_Map_Creator
{
 public:
     Local_Map_Creator();
     void process();

 private:
    //method
    void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr&);
    void current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr&);
    void create_local_map();
    void convert_coordinate(int);
    void convert_grid_map();
    int get_radius(int);
    void add_frame_local_map();
    //parameter
    int hz;
    int width;
    int height;
    double resolution;
    int number_of_laser = 1080;
    int row;
    int column;
    int radius_limit;
    bool is_edge;

    //member
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Publisher pub_local_map;
    ros::Subscriber sub_laser_scan;
    ros::Subscriber sub_current_pose;
    sensor_msgs::LaserScan scan_data;
    nav_msgs::OccupancyGrid local_map;
    std::vector<std::vector<int>> grid_map;
    geometry_msgs::PoseStamped current_pose;
};
#endif//LOCAL_MAP_CREATOR_H
