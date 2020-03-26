#ifndef LOCAL_PATH_PLANNER
#define LOCAL_PATH_PLANNER

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "nav_msgs/Path.h"
#include "tf/tf.h"

class Dynamic_Window_Approch
{
public:
    Dynamic_Window_Approch();
    void process();
private:
    //method
    void local_map_callback(const nav_msgs::OccupancyGrid::ConstPtr&);
    void local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr&);
    void way_blocked_callback(const std_msgs::Bool::ConstPtr&);
    void estimated_pose_callback(const geometry_msgs::PoseStamped::ConstPtr&);
    void create_dynamic_window();
        void spec_window();
        void reachable_window();
        void obstacle_window();
    void consider_local_path();
        void create_virtual_path(double,double,int);
            geometry_msgs::PoseStamped simulate(double,double,double);
        void evaluation(const geometry_msgs::PoseStamped&,int,unsigned int);
            double heading(const geometry_msgs::PoseStamped&);
            double distance(const geometry_msgs::PoseStamped&);
            double velocity(const geometry_msgs::PoseStamped&,int,unsigned int);
     void decide_local_path();

    //parameter
    struct Component
    {
        double linear;
        double angular;
    };
    int hz;
    double dt;//time step
    double dx;//linear step
    double da;//angular step
    double sim_time;
    double max_linear_velocity;
    double max_angular_velocity;
    double max_linear_acceleration;
    double max_angular_acceleration;
    double sigma;
    double k_heading;
    double k_distance;
    double k_velocity;
    bool is_map_recieved = false;
    bool is_goal_recieved = false;

    //member
    struct Considering_list
    {
        nav_msgs::Path virtual_path;
        Component velocity;
        double heading_score;
        double distance_score;
        double velocity_score;
        double total_score;
        bool is_obstacled;
    };
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Publisher pub_roomba_ctrl;
    ros::Subscriber sub_local_map;
    ros::Subscriber sub_local_goal;
    ros::Subscriber sub_estimated_pose;
    ros::Subscriber sub_way_blocked;
    nav_msgs::OccupancyGrid local_map;
    geometry_msgs::PoseStamped local_goal;
    std::vector<Component> window;
    geometry_msgs::PoseStamped current_pose;
    double current_distance;
    geometry_msgs::PoseStamped past_pose;
    Component current_velocity;
    roomba_500driver_meiji::RoombaCtrl roomba_ctrl;
    std::vector<Considering_list> list;

};
#endif//LOCAL_PATH_PLANNER
