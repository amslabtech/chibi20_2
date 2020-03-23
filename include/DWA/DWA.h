#ifndef _DWA_H
#define _DWA_H

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
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

//軌跡を探索するときに(v, omega)の組み合わせを考えるのに使う
struct Speed{
   double v;
   double omega;
};

struct Goal{
    double x;
    double y;
};

struct LaserData{
    double angle;
    double range;
};

struct Dynamic Window{
    double min_v;
    double max_v;
    double min_omega;
    double max_omega;
};

class DWA
{
public:
    DWA();
    void estimatedpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);
    void targetpose_callback(const geometry_msgs::PointStamped::ConstPtr&);
    // void whiteline_callback(const std_msgs::Bool);
    void lasercallback(const sensor_msgs::LaserScan::ConstPtr&);
    void dwa_control(State&,Speed&,Goal,Dynamic Window);
    void process();
private:
    //method
    void motion(State&,Speed);
    double to_goal_cost(std::vector<State>&,Goal,State);
    double calc_goal_dist(std::vector<State>&, Goal);
    double speed_cost(std::vector<State>);
    double obsatcle_cost(State,std::vector<State>);
    void final_input(State,Speed&,Dynamic Window&,Goal);
    void calc_dynamic_window(Dynamic Window&,State&);
    void calc_trajectory(std::vector<State>&, State);
    double calc_to_goal_cost(std::vector<State>&, Goal, State);
    double calc_speed_cost(std::vector<State>);
    double calc_obstacle_cost(State, std::vector<State>);
    void calc_final_input(State, Speed&, Dynamic Window&, Goal);

    //parameter
    double max_speed;
    double min_speed;
    double max_yawrate;
    double max_accel;
    double max_dyawrate;
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
    int hz;
    int N;
    // bool dist;
    LaserData Ldata[N];
    State roomba;
    // {x, y, yaw,v, omega}
    Speed speed;
    Dynamic Window dw;
   // double yaw = 0.0; //temporarily removed
    // bool turn = false; //false = Right, true = Left
    Goal goal = {0, 0};

    //member
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Subscriber laser_sub;
    ros::Subscriber estimated_pose_sub;
    ros::Subscriber target_pose_sub;
    ros::Subscriber whiteline_sub;

    ros::Publisher ctrl_pub;

  //ros::Rate loop_rate();

    roomba_500driver_meiji::RoombaCtrl msg;
    geometry_msgs::PoseWithCovarianceStamped estimated_pose_msg;

};

#endif //_DWA_H
