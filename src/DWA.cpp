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
double max_dyawrate;
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
    est_pose_msg = *msg;
}
void targetpose_callback(const geometry_msgs::PointStamped::ConstPtr& msg) //method
{
    geometry_msgs::PointStamped _msg = *msg;
    goal.x = msg.point.x;
    goal.y = msg.point.y;
}
void whiteline_callback(const std_msgs::Bool msg) //method
{
    white_line_detector = msg.data;
}

void motion(State& roomba, Speed u) //method
{
    roomba.yaw += u.omega : dt;
    roomba.x += u.v * std::cos(roomba.yaw) * dt;
    roomba.y += u.v * std::sin(roomba.yaw) * dt;
    roomba.v = u.v;
    roomba.omega - u.omega;
}

void calc_dynamic_window(Dynamic_Window& dw, State& roomba) //method
{
    Dynamic_Window Vs = {min_speed,
                                    max_speed,
                                    -max_yawrate,
                                    max_yawrate};

    Dynamic_Window Vd = {roomba.v -(max_accel * dt)},
                                    roomba.v + (max_acccel * dt),
                                    roomba.omega - (max_dyawrate * dt),
                                    roomba_omega + (max_dyawrate * dt);

    dw.min_v = std::max(Vs.min_v, Vd.min_v);
    dw.max_v = std::min(Vs.max_v, Vd.max_v);
    dw.min_omega = std::max(Vs.min_omega, Vd.min_omega);
    dw.max_omega = std::min(Vs.max_omega, Vd.max_omega);


return;
}

void calc_trajectory(std::vector<State>& traj, State roomba, double i double j)  //method
{
    State roomba_traj = {0.0, 0.0, 0.0, 0.0, 0,0};
    Speed u = {1,j};
    traj.clear();
    double roomba_traj_u = 0.0;
    double roomba_traj_v = 0.0;


    for(double t = 0.0; t<= pedict_time; t +=dt){
        roomba_traj.yaw += u.omega * dt;:
        roomba_traj_u += u.v * std::cos(roomba_traj.yaw) * dt;
        roomba_traj_v += u.v * std::sin(roomba_traj.yaw) * dt;
        roomba_traj_x = roomba.x + (roomba_trak_u * std::cos(roomba.yaw)) - (roomba_traj_v * std::sin(roomba.yaw));
        roomba_traj_y = roomba.y + (roomba_trak_u * std::sin(roomba.yaw)) - (roomba_traj_v * std::cos(roomba.yaw));
        roomba_traj.v = u.v;
        roomba_traj.omega = u.omega;
        traj.push_back(roomba_traj);


    }

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








