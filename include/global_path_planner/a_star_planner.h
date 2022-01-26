#ifndef A_SATR_PLANNER_H_
#define A_SATR_PLANNER_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "global_path_planner/cost.h"
#include "global_path_planner/coordinate.h"
#include "global_path_planner/waypoint.h"

class AStarPlanner
{
public:
    AStarPlanner();
    void process();

private:
    void map_callback(const nav_msgs::OccupancyGridConstPtr& msg);
    void load_waypoints();
    void load_route();
    void create_waypoints();
    void set_map_parameter();
    void make_limit();
    void expand_wall();
    void add_wall(int x,int y);
    void create_global_path();
    void cretae_waypoint_path(nav_msgs::Path& waypoint_path,int start_index);
    void clean_lists();
    void define_starting_grid(int start_index);
    void define_goal_grid(int goal_index);
    void init_open_list();
    void open_around();
    void open_grid(int x,int y,float move_cost);
    void update_list(int x,int y,float g,float f);
    void update_open_list(int x,int y,float g,float f);
    void clean_close_list(int x,int y);
    void update_close_list();
    void update_searching_grid();
    void check_goal();
    void trace_dealer(nav_msgs::Path& waypoint_path);
    void add_path_point(nav_msgs::Path& waypoint_path,int x,int y);
    void publish_global_path();
    float calc_huristic(int x,int y);


    std::string map_topic_name_;
    std::string path_topic_name_;
    std::string waypoints_topic_name_;

    int row_;
    int column_;
    float resolution_;

    int HZ_;
    int WALL_TH_;
    int WALL_COST_;
    int WALL_RANGE_;

    bool has_received_map_;
    bool is_goal_;
    bool is_start_;

    std::vector<std::vector<int>> grid_map_;
    std::vector<std::vector<Cost>>open_list_;
    std::vector<std::vector<Cost>>close_list_;
    std::vector<Waypoint> waypoints_;
    std::vector<int> routes_;
    XmlRpc::XmlRpcValue waypoints_list_;
    XmlRpc::XmlRpcValue route_list_;

    Coordinate center_;
    Coordinate lower_limit_;
    Coordinate upper_limit_;
    Coordinate goal_grid_;
    Coordinate searching_grid_;
    Coordinate tracing_grid_;

    std::vector<Coordinate> landmarks_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber map_sub_;
    ros::Publisher path_pub_;
    ros::Publisher waypoints_pub_;

    nav_msgs::OccupancyGrid map_;
    nav_msgs::Path global_path_;
};


#endif  // A_SATR_PLANNER_H_
