#include "global_path_planner/global_path_planner.h"

A_Star_Planner::A_Star_Planner() :private_n("~")
{
    //parameter

    //subscriber

    //publisher

}

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    nav_msgs::OccupancyGrid _msg=*msg;
    //std::cout<<"map received"<<std::endl;
    for(int i=0;i<row;i++)
    {
        for(int j=0;j<column;j++)
        {
            grid[i][j]= _msg.data[i*row+j];
        }
    }

    //global_path.header.frame_id="map";

}

void pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
     current_pose=*msg;
}

float A_Star_Planner::huristic(const int& x,const int& y)
{
    return sqrt(pow(x-goal_position.x,2)+pow(y-goal_position.y,2));
}

Costs A_Star_Planner::calc_grid_cost(const Costs& pre_costs)
{
    Costs costs;
    costs.f= pre_costs.g+huristic()

}

Costs A_Star_Planner::calc_start_cost()
{
    Costs costs;
    costs.f=huristic(current_pose.x,current_pose.y);
    return costs;
}

void A_Star_Planner::process()
{

    if(map_callback)//地図データを受け取ったら
    {



int main (int argc, char **argv)
{
  ros::init(argc,argv,"roomba_controller");
  A_Star_Planner a_star_planner;
  a_star_planner.process();
  return 0;
}









