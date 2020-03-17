#ifndef _DWA_H
#define _DWA_H

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometory.h"
#include "math.h"
#include <tf/tf.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


struct State{
    double x;
    double y;
    double yaw;
    double v;
    double omega;
};

//struct Speed{
//    double v;
//    double omega;
//};

struct Goal{
    double x;
    double y;
};

struct LaserData{
    double angle;
    double range;
};

struct Dynamic_Window{
    double min_v;
    double max_v;
    double min_omega;
    double max_omega;
};

class DWA
{
public:
    real_velocity();

private:
    //method
    void estpose_callback(const geometry_msgs::poseWithCovarianceStamped::ConstPtr& msg);
    void targetpose_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void whiteline_callback(const std_msgs::Bool msg);
    void motion();
    void dynamic_window();
    void trajectory();
    double to_goal_cost();
    double goal_dist();
    double speed_cost();
    double obsatcle_cost();
    void final_input();
    void dwa_control();
    void lasercallback(const sensor_msgs::LaserScan::ConstPtr& msg)

    //parameter
    double max_speed;
    double min_speed;
    double max_yawrate;
    double max_accel;
    double max_drawrate;
    double v_reso;
    double yawrate_reso;
    double dt;
    double predict_time;
    double to_goal_cost_gain;
    double dist_gain;
    double speed_cost_gain;
    double obstacle_cost_gain;
    double robot_radius;
    double roomba_v_gain;
    double roomba_omega_gain;

    //member
    ros::NodeHandle roomba_ctrl_pub;
    ros::NodeHandle roomba_odometry_sub;
    ros::NodeHandle scan_laser_sub;
    ros::NodeHandle est_pose;
    ros::NodeHandle target_pose;
    ros::NodeHandle whiteline;
    ros::NodeHandle private_nh;

    ros::Subsciber laser_sub;
    ros::Subsciber est_pose_sub;
    ros::Subsciber target_pose_sub;
    ros::Subsciber whiteline_sub;

    ros::Publisher ctrl_pub;

  //ros::Rate loop_rate();

    geometry_msgs::PoseWithCovarianceStamped est_pose_msg;

};

#endif //_DWA_H
