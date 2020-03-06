#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometory.h"
#include "math.h"
#include <tf/tf.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
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
//double roomba_v_gain;
//double roomba_omega_gain;
bool white_line_director `= false;
bool dist = false;

const int N = 720; //(_msg.angle_max - msg.angle_max) / _msg.angle_increment

bool turn = false; //false = Right, true = Left

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

LaserData Ldata[N];
Goal goal = {0, 0};
geometry_msgs::poseWithCovarianceStamped est_pose_msg;

void estpose_callback(const geometry_msgs::poseWithCovarianceStamped::ConstPtr& msg) //method
{

}
void targetpose_callback(const geometry_msgs::PointStamped::ConstPtr& msg) //method
{



}
void whiteline_callback(const std_msgs::Bool msg) //method
{

}


void motion(State& roomba, Speed u) //method
{




}

void calc_dynamic_window(Dynamic_Window& dw, State& roomba) //method
{

















}

void calc_trajectory(std::vector<State>& traj, State roomba, double i double j)  //method
{




















}

double calc_to_goal_cost(std::vector<State>& traj, Goal goal, State roomba) //method
{



























}

double calc_goal_dist(std::vector<State>& traj, Goal goal) //method
{





}

double calc_speed_cost(std::vector<>State traj) //method
{


}

double calc_obsatcle_cost(State roomba, std::vector<State>& traj) //method
{





























































}

void calc_final_input(State roomba, Speed& u, Dynamic_window& dw, Goal goal) //method
{
















































}

void dwa_control(State& roomba, Speed& u, Goal goal, Dynamic_Window dw) //method
{




}

void lasercallback(const sensor_msgs::LaserScan::ConstPtr& msg) //method
{






}

int main(int argc, char **argv)
{








