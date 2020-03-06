#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D"
#include "nav_msgs/Path"

struct Costs
{
    float g;//g値
    float f;//f値
    int dealer_x;//親のx座標
    int dealer_y;//親のy座標
};

struct Coordinate
{
    int x;
    int y;
};

class A_Star_Planner
{
public:
    A_Star_Planner();

private:
    //method
    float huristic();
    void calc_grid_cost();
    void map_callback(const nav_msgs;;OccupancyGrid::ConstPtr&);
    void pose_callback(const geometry_msgs::Pose2D::ConstPtr& );


    //parameter
    Coordinate goal_position;
    Coordinate start_position;
    Coordinate landmark;
    int wall_cost;
    int grid[row][column];
    Costs open_list[row][column];
    Costs close_list[row][column];
    //member
    static const int row=4000;
    static const int column=4000;
    nav_msgs::OccupancyGrid prior_map;//元マップデータ格納
    nav_msgs::OccupancyGrid updated_map;//経路封鎖時にLocalMapCreaterから受取
    geometry_msgs::Pose2D current_pose;//経路封鎖時のスタート位置用
    nav_msgs::Path global_path;//LocalPathPlannerに出力

    //


};



#endif //GLOBAL_PATH_PLANNER_H
