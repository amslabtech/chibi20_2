#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
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
    void open_grid(const int&,const int&,const int&);
    void update_open_list(const int&,const int&);
    void clean_close_list(const int&,const int&);
    int grid_patern(const int&,const int&);
    void update_close_list();
    void update_searching_grid();
    void check_goal();
    void add_path_point(const int&,const int&);
    void trace_dealer();
    void make_limit();
    //for debagging
    void show_open_list();
    void show_close_list();



    //parameter
    int row;
    int column;
    int scale;
    const float move_cost[8]={1,sqrt(2),1,sqrt(2),1,sqrt(2),1,sqrt(2)};//左から８方向
    int Hz;
    int wall_border;
    int wall_cost;
    bool map_received=false;
    bool reached_goal=false;
    bool reached_start=false;
    float proto_g;
    float proto_f;
    int checkpoint;
    std::vector<std::vector<int>> grid_map;//int grid_map[row][column];
    Coordinate adjust;
    Coordinate goal_grid;
    Coordinate searching_grid;
    Coordinate downlimit;
    Coordinate uplimit;
    const Coordinate landmark[10]={{2000,2000},{1860,2000},{1860,2340},{2000,2340},{2140,2340},{2140,2000},{2140,1680},{2000,1680},{1860,1680},{1860,2000}};//中央、中央左から時計回り{{0,0},{-140,0},{-140,340},{0,340},{140,340},{140,0},{140,-320},{0,-320},{-140,-320},{-140,0}}
    // const Coordinate landmark[10]={{2000,2000},{1845,2000},{1860,2340},{2000,2170},{2140,2340},{2070,2000},{2140,1680},{2000,1650},{1860,1680},{1845,2000}};
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
    //for debagging
    ros::Publisher pub_open_grid;

    //map manager に移す
    ros::Publisher pub_updated_map;
    void map_turn();


};



#endif //GLOBAL_PATH_PLANNER_H
