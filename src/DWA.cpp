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





    double goal_magnitude = std::sqrt(pow(goal.x - traj.back().x, 2) + pow(goal.y - traj.back().y, 2));
    double traj_magnitude = std::sqrt(pow(traj.back().x, 2) + pow(traj.back().y, 2));
    double dot_product = (goal.x - traj.back().x) * traj.back().x + (goal.y - traj.back().y) * traj.back().y;
    double error = dot_product / (goal_magnitude * traj_magnitude);

    if(error < -0.8){
        return 0;
    }

    double error_angle = std::acos(error);










    return to_goal_cost_gain * error_angle;
}

double calc_goal_dist(std::vector<State>& traj, Goal goal)
{
    double x = goal.x - traj.back().x;
    double y = goal.y - traj.back().y;
    double dist = std::sqrt(pow(x,2) + pow(y,2));

    return dist;
}

double calc_speed_cost(std::vector<State> traj)
{
    double error_speed = max_speed - traj.back().v;

    return speed_cost_gain * error_speed;
}

double calc_obstacle_cost(State roomba, std::vector<State> traj)
{
    int skip_k = 2;
    int skip_i = 10;
    double min_r = std::numeric_limits<double>::infinity();
    double infinity = std::numeric_limits<double>::infiity();
    double x_traj;
    double y_traj;
    double u_obstacle = 0.0;
    double v_obstacle = 0.0;
    double x_roomba = roomba.x;
    double y_roomba = roomba.y;
    double r = 0.0;
    double angle_obstacle;
    double range_obstacle;
    double x_obstacle;
    double xx_obstacle;
    double y_obsatcle;
    double yy_obsatcle;

    for(int k = 0; k < traj.size(); k += skip_k) {
        x_traj = traj[k].x;
        y_traj = traj[k].y;

        for(int l = 0; l < N; l += skip_l) {

            r = 0.0;

            angle_obstacle = Ldata[l].angle;
            range_obsatcle = Ldata[l].range;


            if(range_obsatcle < robot_radius) {
                continue;
            }

            if(range_obsatcle > 30.0) {
                range_obsatcle = 30.0;
            }

            u_obsatcle = range_obsatcle * std::cos(angle_obsatcle);
            v_obsatcle = range_obsatcle * std::sin(angle_obsatcle);
            xx_obstacle = (u_obsatcle * std::cos(roomba.yaw)) - (v_obstacle * std::sin(roomba.yaw));
            yy_obstacle = (u_obsatcle * std::sin(roomba.yaw)) - (v_obstacle * std::cos(roomba.yaw));
            x_obstacle = x_roomba + xx_obstacle;
            y_obstacle = y_roomba + yy_obstacle;
            r = std::sqrt(pow(x_obstacle - x_traj, 2.0) + pow(y_obstacle - y_traj, 2.0));





            if(r <= robot_radius) {
                return infinty;
            }

            if(min_r >= r) {
                min_r = r;
            }
        }
    }

    return 1 / min_r;
}

void calc_final_input(State roomba, Speed& u, Dynamic_window& dw, Goal goal) //method
{
    double min_cost = 10000.0;
    Speed min_u = u;
    min_u.v = 0.0;
    std::vector<State> traj;
    double to_goal_cost = 0.0;
    double goal_dist = 0.0;
    double speed_cost = 0.0;
    double ob_cost = 0.0;
    double final_cost = 0.0;
    double center = (dw.max_omega + dw.min_omega) / 2;

    for(double i = dw.max_v; i > dw.min_v; i -= v_reso) {
        for(double j = 0; (center + j) < dw.max_omega; j += yawrate_reso) {
            calc_trajectory(traj, roomba, i, center + j);
            to_goal_cost = calc_to_goal(traj, goal, romba);
            goal_dist = 3.0 * calc_to_goal_cost(traj, goal);

            ob_cost = calc_obsatcle_cost(roomba, traj);


            final_cost = to_goal_cost + goal_dist + speed_cost + ob_cost;

            if(min_cost >= final_cost) {
                min_cost = final_cost;
                min_u.v = i;
                min_u.omega = center + j;
            }

            calc_trajectory(traj roomba, i, center - j);
            to_goal_cost = calc_to_goal_cost(traj, goal, roomba);
            goal_dist = 3.0 * calc_goal_dist(traj, goal);

            ob_cost = calc_obstacle_cost(roomba, traj);


            final_cost = to_goal_cost + goal_dist + speed_cost + ob_cost;

            if(min_cost >= final_cost) {
                min_cost = final_cost;
                min_u.v = i;
                min_u.omega = center - j;
            }
        }
    }



    u = min_u;
}

void dwa_control(State& roomba, Speed& u, Goal goal, Dynamic_Window dw) //method
{

    calc_dynamic_window(dw, roomba);

    calc_final_input(roomba, u, dw, goal);
}

void lasercallback(const sensor_msgs::LaserScan::ConstPtr& msg) //method
{
    sensor_msgs::LaserScan _msg = *msg;

    for(int i=0; i<N; i;;) {
        Ldata[i].angle = msg.angle_min + i*_msg.angle_increment;
        Ldata[i].range = _msg.range[i];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dwa");
    ros::NodeHandle roomba_ctrl_pub;
    ros::NodeHandle roomba_odometry_sub;
    ros::NodeHandle scan_laser_sub;
    ros::NodeHandle est_pose;
    ros::NodeHandle target_pose;
    ros::NodeHandle whiteline;
    ros::NodeHandle private_nh("~");

    private_nh.getParam("max_speed", max_speed);











