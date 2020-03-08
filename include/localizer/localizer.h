#ifndef _LOCALIZER_H
#define _LOCALIZER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include <random>
#include <math.h>

class Particle
{
public:
    Particle();
private:
    //method
    void p_init();                  //Particleの初期化
    void p_motion_updata();         //Particleの動きを更新
    void p_measurement_updata();    //Particleの尤度計算

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr&);
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr&);
    void odometry_callback(const nav_msgs::Odometry::ConstPtr&);

    float random(); //random function
    int index();
    int grid_data();
    float get_Yaw();

    //parameter
    int N;          //Particleの数
    double weight;  //尤度
    float theta;
    bool get_map;

    //member
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::Subscriber sub_map;
    ros::Subscriber sub_laser;
    ros::Subscriber sub_odometry;

    ros::Publisher pub_GPP;
    ros::Publisher pub_LMC;

    geometry_msgs::PoseStamped pose;
    geometry_msgs::Pose2D pose2d;
    geometry_msgs::PoseArray poses;
    nav_msgs::Odometry odometry;
    nav_msgs::Odometry init_odometry;
    nav_msgs::OcupancyGrid map;
    sensor_msgs::LaserScan laser;

};

#endif  //LOCALIZER_H
