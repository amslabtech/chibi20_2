#ifndef
#define

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometory.h"
#include "math.h"
#include <tf/tf.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

class
{
public:


private:
    //method
    void estpose_callback(const geometry_msgs::poseWithCovarianceStamped::ConstPtr& msg)
    void targetpose_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
    void whiteline_callback(const std_msgs::Bool msg)
    void motion(State& roomba, Speed u)
    void calc_dynamic_window(Dynamic_Window& dw, State& roomba)
    void calc_trajectory(std::vector<State>& traj, State roomba, double i double j)
    double calc_to_goal_cost(std::vector<State>& traj, Goal goal, State roomba)
    double calc_goal_dist(std::vector<State>& traj, Goal goal)
    double calc_speed_cost(std::vector<>State traj)
    double calc_obsatcle_cost(State roomba, std::vector<State>& traj)
    void calc_final_input(State roomba, Speed& u, Dynamic_window& dw, Goal goal)
    void dwa_control(State& roomba, Speed& u, Goal goal, Dynamic_Window dw)
    void lasercallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    //parameter
    double max_speed;
    double min_speed;
    double max_yawrate;
    double max_acccel;
    double max_drawrate;
    double v_reso;
    double yawrate_reso;
    double dt;
    double  predict_time;
    double to_goal_cost_gain;
    double dist_gain;
    double speed_cost_gain;
    double obstacle_cost_gain;
    double robot_radius;
    //member

};

#endif
