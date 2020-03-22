#include "DWA/DWA.h"

// const int N = 720;
// LaserData Ldata[N];
// Goal goal = {0, 0};

//初期化
DWA::DWA() :private_nh("~")
{
    const int N = 720;
              //    {x,   y, yaw,   v, omega}
    State roomba ={0.0, 0.0, 0.0, 0.0, 0.0};
              //    {v, omega}
    Speed speed = {0.0, 0.0};
              //      {min_v, max_v, min_omega, max_omega}
    Dynamic_Window dw = {0.0, 0.0, 0.0, 0.0};
              //{x. y}
    Goal goal = {0, 0};

    private_nh.param("max_speed", max_speed,{0.5});
    private_nh.param("min_speed", min_speed,{-0.5});
    private_nh.param("max_yawrate", max_yawrate,{0.5});
    private_nh.param("max_accel", max_accel,{0.5});
    private_nh.param("max_dyawrate", max_dyawrate,{0.05});
    private_nh.param("v_reso", v_reso,{0.1});
    private_nh.param("yawrate_reso", yawrate_reso,{0.1});
    private_nh.param("dt", dt,{0.1});
    private_nh.param("predict_time", predict_time,{2.0});
    private_nh.param("to_goal_cost_gain", to_goal_cost_gain,{1.0});
    private_nh.param("speed_cost_gain", speed_cost_gain,{1.0});
    private_nh.param("robot_radius", robot_radius,{0.34});
    private_nh.param("roomba_v_gain", roomba_v_gain,{2.0});
    private_nh.param("roomba_omega_gain", roomba_omega_gain,{2.0});
    private_nh.param("hz", hz,{10});

   // Subscriber
    laser_sub = nh.subscribe("scan", 1,&DWA::lasercallback, this);
    estimated_pose_sub = nh.subscribe("chibi20_2/estimated_pose", 1,&DWA::estimatedpose_callback, this);
    target_pose_sub = nh.subscribe("chibi20_2/target", 1, &DWA::targetpose_callback, this);
   // Publisher
    ctrl_pub = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba/control", 1);
}

//localizationから最新の情報を受け取るとspinonce, 予測経路を他の関数に引数としてわたす
void DWA::estimatedpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) //method
{
    estimated_pose_msg = *msg;
}

//目的の位置情報を引数としてわたす
void DWA::targetpose_callback(const geometry_msgs::PointStamped::ConstPtr& msg) //method
{
    geometry_msgs::PointStamped _msg = *msg;
    goal.x = _msg.point.x;
    goal.y = _msg.point.y;
}

//ルンバの動きと状態, local mapと被る
void DWA::motion(State& roomba, Speed speed) //method
{
    roomba.yaw += speed.omega * dt;
    roomba.x += speed.v * std::cos(roomba.yaw) * dt;
    roomba.y += speed.v * std::sin(roomba.yaw) * dt;
    roomba.v = speed.v;
    roomba.omega = speed.omega;
}
//ダイナミックウィンドウを作る
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

//軌道を探索 //i,j need revised to vector
void DWA::calc_trajectory(std::vector<State>& traj, State roomba, double i, double j)
{                      // {x,   y, yaw,   v, omega}
    State roomba_traj = {0.0, 0.0, 0.0, 0.0, 0.0};
    Speed speed = {i,j};
    traj.clear();
    double roomba_traj_speed = 0.0;
    double roomba_traj_v = 0.0;

    for(double t = 0.0; t<= predict_time; t +=dt){
        roomba_traj.yaw += speed.omega * dt;
        roomba_traj_speed += speed.v * std::cos(roomba_traj.yaw) * dt;
        roomba_traj_v += speed.v * std::sin(roomba_traj.yaw) * dt;
        roomba_traj.x = roomba.x + (roomba_traj_speed * std::cos(roomba.yaw)) - (roomba_traj_v * std::sin(roomba.yaw));
        roomba_traj.y = roomba.y + (roomba_traj_speed * std::sin(roomba.yaw)) - (roomba_traj_v * std::cos(roomba.yaw));
        roomba_traj.v = speed.v;
        roomba_traj.omega = speed.omega;
        traj.push_back(roomba_traj);
    }
}

//目的地にかけるコストを算出
double DWA::calc_to_goal_cost(std::vector<State>& traj, Goal goal, State roomba)
{
    //ゴールと最終位置との距離
    double goal_length = std::sqrt(pow(goal.x - traj.back().x, 2) + pow(goal.y - traj.back().y, 2));

    //最終位置とロボットとの距離
    double traj_length = std::sqrt(pow(traj.back().x, 2) + pow(traj.back().y, 2));

    //内積の計算
    double dot_product = (goal.x - traj.back().x) * traj.back().x + (goal.y - traj.back().y) * traj.back().y;

    //errorはcos(theta)の値
    double error = dot_product / (goal_length * traj_length);

    if(error < -0.8){
        return 0;
    }

    double error_angle = std::acos(error);

    return to_goal_cost_gain * error_angle;
}
//目的地までの距離を計算
double DWA::calc_goal_dist(std::vector<State>& traj, Goal goal)
{
    double x = goal.x - traj.back().x;
    double y = goal.y - traj.back().y;
    double distance = std::sqrt(pow(x,2) + pow(y,2));

    return distance;
}

double DWA::calc_speed_cost(std::vector<State> traj)
{
    double error_speed = max_speed - traj.back().v;

    return speed_cost_gain * error_speed;
}

//障害物に近いほどコストを与える
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

//選出した速度, 加速度をロボットに代入
void DWA::calc_final_input(State roomba, Speed& speed, Dynamic_Window& dw, Goal goal)
{
    double min_cost = 1e8;
    Speed min_speed = speed;
    min_speed.v = 0.0;
    std::vector<State> traj;
    double to_goal_cost = 0.0;
    double goal_dist = 0.0;
    double speed_cost = 0.0;
    double obstacle_cost = 0.0;
    double final_cost = 0.0;
    double center = (dw.max_omega + dw.min_omega) / 2;

    for(double i = dw.max_v; i > dw.min_v; i -= v_reso) {
        for(double j = 0; (center + j) < dw.max_omega; j += yawrate_reso) {
            calc_trajectory(traj, roomba, i, center + j);
            to_goal_cost = calc_to_goal_cost(traj, goal, roomba);

//mazical number, 3.0 needs altering to gain;
            goal_dist = 3.0 * calc_goal_dist(traj, goal);
            obstacle_cost = calc_obstacle_cost(roomba, traj);
            final_cost = to_goal_cost + goal_dist + speed_cost + obstacle_cost;

            if(min_cost >= final_cost) {
                min_cost = final_cost;
                min_speed.v = i;
                min_speed.omega = center + j;
            }

            calc_trajectory(traj, roomba, i, center - j);
            to_goal_cost = calc_to_goal_cost(traj, goal, roomba);
            goal_dist = 3.0 * calc_goal_dist(traj, goal);

            obstacle_cost = calc_obstacle_cost(roomba, traj);


            final_cost = to_goal_cost + goal_dist + speed_cost + obstacle_cost;

            if(min_cost >= final_cost) {
                min_cost = final_cost;
                min_speed.v = i;
                min_speed.omega = center - j;
            }
        }
    }
    speed = min_speed;
}
//DWAの実行
void DWA::dwa_control(State& roomba, Speed& speed, Goal goal, Dynamic_Window dw)
{
    calc_dynamic_window(dw, roomba);

    calc_final_input(roomba, speed, dw, goal);
}
//センサからの情報を引数としてわたす
void DWA::lasercallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_msgs::LaserScan _msg = *msg;

    for(int i=0; i<N; i++) {
        Ldata[i].angle = _msg.angle_min + i*_msg.angle_increment;
        Ldata[i].range = _msg.ranges[i];
    }
}

void DWA::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
    ros::spinOnce();

    //motion(roomba, speed);
    roomba.yaw = tf::getYaw(estimated_pose_msg.pose.pose.orientation);
    roomba.v = speed.v;
    roomba.omega = speed.omega;
    roomba.x = estimated_pose_msg.pose.pose.position.x;
    roomba.y = estimated_pose_msg.pose.pose.position.y;

    dwa_control(roomba, speed, goal, dw);

    //ゴールの方に向いているか
    msg.cntl.linear.x = roomba_v_gain * speed.v / max_speed;
    msg.cntl.angular.z = roomba_omega_gain * speed.v / max_yawrate;
    }
}
    //ゴール判別
    // if(dist == false && roomba.x > 2.0) {
        // dist = true;
    // }

    //check goal

    //ROS_INFO("x = %f, goal_dist = %f", roomba.x, sqrt(pow(roomba.x - goal.x, 2.0) + pow(roomba.y - goal.y, 2.0)));

    //if(dist == true && sqrt(pow(roomba.x - goal.x, 2.0) + pow(roomba.y - goal.y, 2.0)) < 0.4) {
    //    ROS_INFO("Goal");
    //    msg.cntl.linear.x = 0.0;
    //    msg.cntl.angular.z = 0.0;
    //    msg.mode = 0;
    //    ctrl_pub.publish(msg);
    //}
    // msg.mode = 11;
    // ctrl_pub.publish(msg);
    // loop_rate.sleep();
    // }
// }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dwa");
    std::cout<<"test now"<<std::endl;
    DWA dwa;
    dwa.process();
    return 0;
}



















