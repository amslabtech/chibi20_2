#ifndef LOCAL_GOAL_CREATOR_H
#define LOCAL_GOAL_CREATOR_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

class Local_Goal_Creator
{
public:
    Local_Goal_Creator();
    void process();

private:
    //method
    void global_path_callback(const nav_msgs::Path::ConstPtr&);
    void current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr&);
    void select_next_goal();
    //parameter
    int hz;
    double border_distance;

    //member
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Publisher pub_local_goal;
    ros::Subscriber sub_global_path;
    ros::Subscriber sub_current_pose;
    nav_msgs::Path global_path;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped local_goal;
    unsigned int goal_number;
//visualization_msgs/Maker rvizに配信するとき色とかかたちとか変えられる
};

#endif//LOCAL_GOAL_CREATOR_H
