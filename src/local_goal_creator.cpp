#include "local_goal_creator/local_goal_creator.h"

Local_Goal_Creator::Local_Goal_Creator():private_nh("~")
{
    //parameter
    private_nh.param("hz",hz,{1});
    private_nh.param("border_distance",border_distance,{5.0});
    //subscriber
    sub_global_path = nh.subscribe("global_path",10,&Local_Goal_Creator::global_path_callback,this);
    sub_current_pose = nh.subscribe("estimate_pose",10,&Local_Goal_Creator::current_pose_callback,this);
    //publisher
    pub_local_goal = nh.advertise<geometry_msgs::PoseStamped>("local_goal",1);
}

void Local_Goal_Creator::global_path_callback(const nav_msgs::Path::ConstPtr& msg)
{
    global_path=*msg;
    goal_number = 0;
    local_goal = global_path.poses[goal_number];
}

void Local_Goal_Creator::current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}


void Local_Goal_Creator::select_next_goal()
{
    double measure_distance = sqrt(pow(local_goal.pose.position.x-current_pose.pose.position.x,2)+pow(local_goal.pose.position.y-current_pose.pose.position.y,2));
    if(measure_distance < border_distance) goal_number++;
    if(global_path.poses.size() < goal_number) local_goal = global_path.poses[goal_number];
}

void Local_Goal_Creator::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        select_next_goal();
        local_goal.header.frame_id = "map";
        pub_local_goal.publish(local_goal);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main (int argc,char **argv)
{
    ros::init(argc, argv, "local_goal_planner");
    Local_Goal_Creator local_goal_planner;
    std::cout<<"local_goal_planner has started"<<std::endl;
    local_goal_planner.process();
    return 0;
}

