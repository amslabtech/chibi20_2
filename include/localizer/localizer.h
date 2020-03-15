#ifndef _LOCALIZER_H
#define _LOCALIZER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
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
    void p_init(double,double,double,double,double,double);
    void p_motion_update(geometry_msgs::PoseStamped,geometry_msgs::PoseStamped);
    void p_measurement_update();
    void p_move(double,double,double);
    void process();

    geometry_msgs::PoseStamped pose;
    double weight;

private:
    //method
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr&);
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr&);
    void odometry_callback(const nav_msgs::Odometry::ConstPtr&);

    int index(double,double);                   //indexの計算
    int grid_data(double,double);               //Griddataの取得
    double get_Yaw(geometry_msgs::Quaternion);  //Yaw取得
    double angle_diff(double,double);           //角度差の算出
    double get_Range(double,double,double);     //Rangeの更新
    bool judge_update(double,double);           //更新するかしないかの判断
    void create_new_cov(double*,double*,double*);

    //parameter
    int N;                  //Particleの数
    double INIT_X;
    double INIT_Y;
    double INIT_YAW;
    double INIT_X_COV;
    double INIT_Y_COV;
    double INIT_YAW_COV;
    double MAX_RANGE;
    int RANGE_STEP;
    double X_TH;
    double Y_TH;
    double YAW_TH;
    double P_COV;
    double MOVE_X_COV;
    double MOVE_Y_COV;
    double MOVE_YAW_COV;
    double ALPHA_SLOW;
    double ALPHA_FAST;

    //member
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Subscriber map_sub;
    ros::Subscriber lsr_sub;
    ros::Subscriber odo_sub;

    ros::Publisher lmc_sub;
    ros::Publisher gpp_sub;

    geometry_msgs::PoseStamped estimated_pose;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped previous_pose;
    geometry_msgs::Pose2D pose2d;
    geometry_msgs::PoseArray poses;
    nav_msgs::Odometry odometry;
    nav_msgs::OccupancyGrid map;
    sensor_msgs::LaserScan laser;

};

#endif  //LOCALIZER_H
