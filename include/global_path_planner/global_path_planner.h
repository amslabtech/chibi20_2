#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Path.h"

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
    void process();

private:
    //method
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr&);
    void pose_callback(const geometry_msgs::Pose2D::ConstPtr& );
    void path_creator();
    void clean_lists();
    float huristic(const int&,const int&);
    void define_starting_grid();
    void define_goal_grid();
    void open_around();
    void update_open_list(const int&,const int&);
    void clean_close_list(const int&,const int&);
    int grid_patern(const int&,const int&);
    void update_close_list();
    void update_searching_grid();
    void check_goal();
    void add_path_point(const int&,const int&);
    void trace_dealer();



    //parameter
    int row;
    int column;
    const int move_cost[4]={1,1,1,1};//左上右下の順番
    int Hz;
    int wall_border;
    int wall_cost;
    bool map_received=false;
    bool reached_goal=false;
    bool reached_start=false;
    float proto_g;
    float proto_f;
    std::vector<std::vector<int>> grid_map;//int grid_map[row][column];
    Coordinate adjust;
    Coordinate goal_grid;
    Coordinate searching_grid;
    Coordinate landmark[4];
    Coordinate tracing_grid;
    std::vector<std::vector<Costs>>open_list;//Costs open_list[row][column];
    std::vector<std::vector<Costs>>close_list;//Costs close_list[row][column];


    //member
    ros::NodeHandle n;
    ros::NodeHandle private_n;
    ros::Publisher pub_path;
    ros::Subscriber sub_map;
    ros::Subscriber sub_map_metadata;
    ros::Subscriber sub_pose;
    nav_msgs::OccupancyGrid prior_map;//元マップデータ格納
    nav_msgs::MapMetaData map_metadata;
    nav_msgs::OccupancyGrid updated_map;//経路封鎖時にLocalMapCreaterから受取
    geometry_msgs::Pose2D current_pose;//経路封鎖時のスタート位置用
    nav_msgs::Path global_path;//LocalPathPlannerに出力

    //


};



#endif //GLOBAL_PATH_PLANNER_H
