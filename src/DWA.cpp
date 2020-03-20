#include "DWA.h"

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
bool white_line_detector = false;
bool dist = false;

const int N = 720; //(_msg.angle_max - msg.angle_max) / _msg.angle_increment

bool turn = false; //false = Right, true = Left

LaserData Ldata[N];
Goal goal = {0, 0};
geometry_msgs::PoseWithCovarianceStamped est_pose_msg;

void DWA::estpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) //method
{
    est_pose_msg = *msg;
}
void DWA::targetpose_callback(const geometry_msgs::PointStamped::ConstPtr& msg) //method
{
    geometry_msgs::PointStamped _msg = *msg;
    goal.x = _msg.point.x;
    goal.y = _msg.point.y;
}
void DWA::whiteline_callback(const std_msgs::Bool msg) //method
{
    white_line_detector = msg.data;
}

void DWA::motion(State& roomba, Speed u) //method
{
    roomba.yaw += u.omega * dt;
    roomba.x += u.v * std::cos(roomba.yaw) * dt;
    roomba.y += u.v * std::sin(roomba.yaw) * dt;
    roomba.v = u.v;
    roomba.omega = u.omega;
}

void DWA::calc_dynamic_window(Dynamic_Window& dw, State& roomba) //method
{
    Dynamic_Window Vs = {min_speed,
                                    max_speed,
                                    -max_yawrate,
                                    max_yawrate};

    Dynamic_Window Vd = {roomba.v -(max_accel * dt),
                                    roomba.v + (max_accel * dt),
                                    roomba.omega - (max_dyawrate * dt),
                                    roomba.omega + (max_dyawrate * dt)};

    dw.min_v = std::max(Vs.min_v, Vd.min_v);
    dw.max_v = std::min(Vs.max_v, Vd.max_v);
    dw.min_omega = std::max(Vs.min_omega, Vd.min_omega);
    dw.max_omega = std::min(Vs.max_omega, Vd.max_omega);


return;
}

void DWA::calc_trajectory(std::vector<State>& traj, State roomba, double i, double j)  //method
{
    State roomba_traj = {0.0, 0.0, 0.0, 0.0, 0.0};
    Speed u = {i,j};
    traj.clear();
    double roomba_traj_u = 0.0;
    double roomba_traj_v = 0.0;


    for(double t = 0.0; t<= predict_time; t +=dt){
        roomba_traj.yaw += u.omega * dt;
        roomba_traj_u += u.v * std::cos(roomba_traj.yaw) * dt;
        roomba_traj_v += u.v * std::sin(roomba_traj.yaw) * dt;
        roomba_traj.x = roomba.x + (roomba_traj_u * std::cos(roomba.yaw)) - (roomba_traj_v * std::sin(roomba.yaw));
        roomba_traj.y = roomba.y + (roomba_traj_u * std::sin(roomba.yaw)) - (roomba_traj_v * std::cos(roomba.yaw));
        roomba_traj.v = u.v;
        roomba_traj.omega = u.omega;
        traj.push_back(roomba_traj);


    }

}

double DWA::calc_to_goal_cost(std::vector<State>& traj, Goal goal, State roomba) //method
{
    // calculation for inner product




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

double DWA::calc_goal_dist(std::vector<State>& traj, Goal goal)
{
    double x = goal.x - traj.back().x;
    double y = goal.y - traj.back().y;
    double dist = std::sqrt(pow(x,2) + pow(y,2));

    return dist;
}

double DWA::calc_speed_cost(std::vector<State> traj)
{
    double error_speed = max_speed - traj.back().v;

    return speed_cost_gain * error_speed;
}

double DWA::calc_obstacle_cost(State roomba, std::vector<State> traj)
{
    int skip_k = 2;
    int skip_l = 10;
    double min_r = std::numeric_limits<double>::infinity();
    double infinity = std::numeric_limits<double>::infinity();
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
    double y_obstacle;
    double yy_obstacle;

    for(unsigned int k = 0; k < traj.size(); k += skip_k) {
        x_traj = traj[k].x;
        y_traj = traj[k].y;

        for(int l = 0; l < N; l += skip_l) {

            r = 0.0;

            angle_obstacle = Ldata[l].angle;
            range_obstacle = Ldata[l].range;


            if(range_obstacle < robot_radius) {
                continue;
            }

            if(range_obstacle > 30.0) {
                range_obstacle = 30.0;
            }

            u_obstacle = range_obstacle * std::cos(angle_obstacle);
            v_obstacle = range_obstacle * std::sin(angle_obstacle);
            xx_obstacle = (u_obstacle * std::cos(roomba.yaw)) - (v_obstacle * std::sin(roomba.yaw));
            yy_obstacle = (u_obstacle * std::sin(roomba.yaw)) - (v_obstacle * std::cos(roomba.yaw));
            x_obstacle = x_roomba + xx_obstacle;
            y_obstacle = y_roomba + yy_obstacle;
            r = std::sqrt(pow(x_obstacle - x_traj, 2.0) + pow(y_obstacle - y_traj, 2.0));





            if(r <= robot_radius) {
                return infinity;
            }

            if(min_r >= r) {
                min_r = r;
            }
        }
    }

    return 1 / min_r;
}

void DWA::calc_final_input(State roomba, Speed& u, Dynamic_Window& dw, Goal goal) //method
{
    double min_cost = 100000000.0;
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
            to_goal_cost = calc_to_goal_cost(traj, goal, roomba);
            goal_dist = 3.0 * calc_goal_dist(traj, goal);

            ob_cost = calc_obstacle_cost(roomba, traj);


            final_cost = to_goal_cost + goal_dist + speed_cost + ob_cost;

            if(min_cost >= final_cost) {
                min_cost = final_cost;
                min_u.v = i;
                min_u.omega = center + j;
            }

            calc_trajectory(traj, roomba, i, center - j);
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

void DWA::dwa_control(State& roomba, Speed& u, Goal goal, Dynamic_Window dw) //method
{

    calc_dynamic_window(dw, roomba);

    calc_final_input(roomba, u, dw, goal);
}

void DWA::lasercallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_msgs::LaserScan _msg = *msg;

    for(int i=0; i<N; i++) {
        Ldata[i].angle = _msg.angle_min + i*_msg.angle_increment;
        Ldata[i].range = _msg.ranges[i];
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
    private_nh.getParam("min_speed", min_speed);
    private_nh.getParam("max_yawrate", max_yawrate);
    private_nh.getParam("max_accel", max_accel);
    private_nh.getParam("max_dyawrate", max_dyawrate);
    private_nh.getParam("v_reso", v_reso);
    private_nh.getParam("yawrate_reso", yawrate_reso);
    private_nh.getParam("dt", dt);
    private_nh.getParam("predict_time", predict_time);
    private_nh.getParam("to_goal_cost_gain", to_goal_cost_gain);
    private_nh.getParam("speed_cost_gain", speed_cost_gain);
    private_nh.getParam("robot_radius", robot_radius);
    private_nh.getParam("roomba_v_gain", roomba_v_gain);
    private_nh.getParam("roomba_omega_gain", roomba_omega_gain);

    ros::Publisher ctrl_pub = roomba_ctrl_pub.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba/control", 1);
    ros::Subscriber laser_sub = scan_laser_sub.subscribe("scan", 1, lasercallback);
    ros::Subscriber est_pose_sub = est_pose.subscribe("chibi20_2/estimated_pose", 1, estpose_callback);
    ros::Subscriber target_pose_sub = target_pose.subscribe("chibi20_2/target", 1, targetpose_callback);
    ros::Subscriber whiteline_sub = whiteline.subscribe("whiteline", 1, whiteline_callback);
    ros::Rate loop_rate(10);

    roomba_500driver_meiji::RoombaCtrl msg;

    msg.mode = 11;

    State roomba ={0.0, 0.0, 0.0, 0.0, 0.0};
    // {x, y, yaw,v, omega}
    Speed u = {0.0, 0.0};
    Dynamic_Window dw = {0.0, 0.0, 0.0, 0.0};
   // double yaw = 0.0; //temporarily removed

    while(ros::ok())
    {
    ros::spinOnce();

    //motion(roomba, u);
    roomba.yaw = tf::getYaw(est_pose_msg.pose.pose.orientation);
    roomba.v = u.v;
    roomba.omega = u.omega;
    roomba.x = est_pose_msg.pose.pose.position.x;
    roomba.y = est_pose_msg.pose.pose.position.y;

    dwa_control(roomba, u, goal, dw);

    msg.cntl.linear.x = roomba_v_gain * u.omega / max_yawrate;
    if(fabs(msg.cntl.angular.z) < 0.13) { //fabs is absolute value
        if(-0.13 < msg.cntl.angular.z && msg.cntl.angular.z <-0.5) {
            msg.cntl.angular.z = -0.13;
        }
        else if(0.5 < msg.cntl.angular.z && msg.cntl.angular.z < 0.13) {
            msg.cntl.angular.z = 0.13;
        }
        else{
            msg.cntl.angular.z = 0.0;
        }
    }

    while(white_line_detector == true) {

        ROS_INFO("white_line");

        msg.cntl.linear.x = 0.0;
        msg.cntl.angular.z = 0.0;
        ctrl_pub.publish(msg);
        sleep(5);

        ROS_INFO("Restart");

        msg.cntl.linear.x = 0.2;
        msg.cntl.angular.z = 0.0;
        ctrl_pub.publish(msg);
        sleep(1);

        break;
    }

    if(dist == false && roomba.x > 2.0) {
        dist = true;
    }

    //check goal

    //ROS_INFO("x = %f, goal_dist = %f", roomba.x, sqrt(pow(roomba.x - goal.x, 2.0) + pow(roomba.y - goal.y, 2.0)));

    if(dist == true && sqrt(pow(roomba.x - goal.x, 2.0) + pow(roomba.y - goal.y, 2.0)) < 0.4) {
        ROS_INFO("Goal");
        msg.cntl.linear.x = 0.0;
        msg.cntl.angular.z = 0.0;
        msg.mode = 0;
        ctrl_pub.publish(msg);
        loop_rate.sleep();
        return 0;
    }

    ctrl_pub.publish(msg);




    loop_rate.sleep();
    }

    return 0;
}





















