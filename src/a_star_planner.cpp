#include "global_path_planner/a_star_planner.h"

AStarPlanner::AStarPlanner() :
    private_nh_("~"),
    has_received_map_(false), is_goal_(false), is_start_(false)
{
    private_nh_.param("map_topic_name",map_topic_name_,{"map"});
    private_nh_.param("path_topic_name",path_topic_name_,{"global_path"});
    private_nh_.param("waypoints_topic_name",waypoints_topic_name_,{"waypoints"});

    private_nh_.param("HZ",HZ_,{1});
    private_nh_.param("WALL_TH",WALL_TH_,{50});
    private_nh_.param("WALL_COST",WALL_COST_,{(int)1e8});
    private_nh_.param("WALL_RANGE",WALL_RANGE_,{4});

    map_sub_ = nh_.subscribe(map_topic_name_,10,&AStarPlanner::map_callback,this);
    path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_name_,1);
    waypoints_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(waypoints_topic_name_,1);

    load_waypoints();
    load_route();
    create_waypoints();
}

void AStarPlanner::map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    if(has_received_map_) return;
    map_ = *msg;
    global_path_.header.frame_id = msg->header.frame_id;
    set_map_parameter();
    make_limit();
    expand_wall();
    create_global_path();
    has_received_map_ =  true;
}

void AStarPlanner::load_waypoints()
{
    if(!private_nh_.getParam("waypoints_list",waypoints_list_)){
        ROS_WARN("Cloud not load waypoints list");
        return;
    }
    ROS_ASSERT(waypoints_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i = 0; i < (int)waypoints_list_.size(); i++){
        if(!waypoints_list_[i]["id"].valid() || !waypoints_list_[i]["x"].valid() || !waypoints_list_[i]["y"].valid()){
            ROS_WARN("waypoints list is valid");
            return;
        }
        if(waypoints_list_[i]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt && waypoints_list_[i]["x"].getType() == XmlRpc::XmlRpcValue::TypeInt && waypoints_list_[i]["y"].getType() == XmlRpc::XmlRpcValue::TypeInt){
            int id = static_cast<int>(waypoints_list_[i]["id"]);
            int x = static_cast<int>(waypoints_list_[i]["x"]);
            int y = static_cast<int>(waypoints_list_[i]["y"]);
            Waypoint waypoint(id,x,y);
            waypoints_.push_back(waypoint);
        }
    }
}

void AStarPlanner::load_route()
{
    if(!private_nh_.getParam("route_list",route_list_)){
        ROS_WARN("Cloud not load route list");
        return;
    }
    ROS_ASSERT(route_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i = 0; i < (int)route_list_.size(); i++){
        if(!route_list_[i]["id"].valid()){
            ROS_WARN("route list is valid");
            return;
        }
        if(route_list_[i]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt){
            int id = static_cast<int>(route_list_[i]["id"]);
            routes_.push_back(id);
        }
    }
}

void AStarPlanner::create_waypoints()
{
    for(const auto & id : routes_){
        for(const auto & wp : waypoints_){
            if(id == wp.id){
                Coordinate coordinate(wp.x,wp.y);
                landmarks_.push_back(coordinate);
            }
        }
    }
}

void AStarPlanner::set_map_parameter()
{
    row_ = map_.info.height;
    column_ = map_.info.width;
    resolution_ = map_.info.resolution;
    center_.set_parameter(row_/2,column_/2);

    grid_map_.resize(row_,std::vector<int>(column_,-1));
    open_list_.resize(row_,std::vector<Cost>(column_));
    close_list_.resize(row_,std::vector<Cost>(column_));
    for(int i = 0; i < row_; i++){
        for(int j = 0; j < column_; j++){
            grid_map_[i][j] = map_.data[j*row_ + i];
        }
    }
}


void AStarPlanner::make_limit()
{
    lower_limit_.set_parameter(row_,column_);
    upper_limit_.set_parameter(0,0);
    for(int i = 0; i < row_; i++){
        for(int j = 0; j < column_; j++){
            if(grid_map_[i][j] != -1){
                if(i < lower_limit_.x) lower_limit_.x = i;
                if(j < lower_limit_.y) lower_limit_.y = j;
                if(i > upper_limit_.x) upper_limit_.x = i;
                if(j > upper_limit_.y) upper_limit_.y = j;
            }
        }
    }
}

void AStarPlanner::expand_wall()
{
    for(int i = lower_limit_.x; i < upper_limit_.x; i++){
        for (int j = lower_limit_.y; j < upper_limit_.y; j++){
            if(grid_map_[i][j] == 100) add_wall(i,j);
        }
    }
}

void AStarPlanner::add_wall(int x,int y)
{
    for(int i = x - WALL_RANGE_; i <= x + WALL_RANGE_; i++){
        for(int j = y - WALL_RANGE_; j <= y + WALL_RANGE_; j++){
            if(grid_map_[i][j] != -1 && grid_map_[i][j] != 100) grid_map_[i][j] = 99;
        }
    }
}

void AStarPlanner::create_global_path()
{
    for(int i = 0; i < (int)landmarks_.size() - 1; i++){
        nav_msgs::Path waypoint_path;
        cretae_waypoint_path(waypoint_path,i);
        global_path_.poses.insert(global_path_.poses.end(),waypoint_path.poses.begin(),waypoint_path.poses.end());
    }
}

void AStarPlanner::cretae_waypoint_path(nav_msgs::Path& waypoint_path,int start_index)
{
    clean_lists();
    define_starting_grid(start_index);
    define_goal_grid(start_index + 1);
    init_open_list();
    while(!is_goal_){
        open_around();
        update_close_list();
        update_searching_grid();
        check_goal();
    }
    trace_dealer(waypoint_path);
    std::reverse(waypoint_path.poses.begin(),waypoint_path.poses.end());
}

void AStarPlanner::clean_lists()
{
    for(int i = lower_limit_.x; i < upper_limit_.x; i++){
        for(int j = lower_limit_.y; j < upper_limit_.y; j++){
            close_list_[i][j].set_parameter(WALL_COST_,WALL_COST_,-1,-1);
            open_list_[i][j].set_parameter(WALL_COST_,WALL_COST_,-1,-1);
        }
    }
}

void AStarPlanner::define_starting_grid(int start_index)
{
    searching_grid_.set_parameter(landmarks_[start_index].x,landmarks_[start_index].y);
    is_goal_ = false;
}

void AStarPlanner::define_goal_grid(int goal_index)
{
    goal_grid_.set_parameter(landmarks_[goal_index].x,landmarks_[goal_index].y);
}

void AStarPlanner::init_open_list()
{
    open_list_[searching_grid_.x][searching_grid_.y].set_parameter(0.0,
                                                                   calc_huristic(searching_grid_.x,searching_grid_.y),
                                                                   searching_grid_.x,
                                                                   searching_grid_.y);
}

void AStarPlanner::open_around()
{
    for(int i = -1; i <= 1; i++){
        for(int j = -1; j <= 1; j++){
            if(i == 0 && j == 0) continue;
            open_grid(i,j,std::sqrt(i*i + j*j));
        }
    }
}

void AStarPlanner::open_grid(int x,int y,float move_cost)
{
    float next_g = open_list_[searching_grid_.x][searching_grid_.y].g + move_cost;
    float next_f = next_g + calc_huristic(searching_grid_.x + x,searching_grid_.y + y);

    update_list(x,y,next_g,next_f);
}

void AStarPlanner::update_list(int x,int y,float g,float f)
{
    // This grid is in the open list
    if(open_list_[searching_grid_.x + x][searching_grid_.y + y].f < WALL_COST_){
        if(f < open_list_[searching_grid_.x + x][searching_grid_.y + y].f) update_open_list(x,y,g,f);
    }
    else{
        // This grid is the close list
        if(close_list_[searching_grid_.x + x][searching_grid_.y + y].f < WALL_COST_){
            if(f < close_list_[searching_grid_.x + x][searching_grid_.y + y].f){
                update_open_list(x,y,g,f);
                clean_close_list(x,y);
            }
        }
        // This grid isn't in any lists
        else update_open_list(x,y,g,f);
    }
}

void AStarPlanner::update_open_list(int x,int y,float g,float f)
{
    if(grid_map_[searching_grid_.x + x][searching_grid_.y + y] > WALL_TH_ || grid_map_[searching_grid_.x + x][searching_grid_.y + y] == -1) return;
    open_list_[searching_grid_.x + x][searching_grid_.y + y].set_parameter(g,f,searching_grid_.x,searching_grid_.y);
}

void AStarPlanner::clean_close_list(int x,int y)
{
    close_list_[searching_grid_.x + x][searching_grid_.y + y].set_parameter(WALL_COST_,WALL_COST_,-1,-1);
}

void AStarPlanner::update_close_list()
{
    close_list_[searching_grid_.x][searching_grid_.y].set_parameter(open_list_[searching_grid_.x][searching_grid_.y].g,
                                                                    open_list_[searching_grid_.x][searching_grid_.y].f,
                                                                    open_list_[searching_grid_.x][searching_grid_.y].parent_x,
                                                                    open_list_[searching_grid_.x][searching_grid_.y].parent_y);
    open_list_[searching_grid_.x][searching_grid_.y].set_parameter(WALL_COST_,WALL_COST_,-1,-1);
}

void AStarPlanner::update_searching_grid()
{
    Cost next_grid;
    next_grid.set_parameter(WALL_COST_,WALL_COST_,-1,-1);
    for(int i = lower_limit_.x; i < upper_limit_.x; i++){
        for(int j = lower_limit_.y; j < upper_limit_.y; j++){
            if(open_list_[i][j].f < next_grid.f){
                next_grid.set_parameter(open_list_[i][j].g,open_list_[i][j].f,i,j);
            }
        }
    }

    if(next_grid.parent_x == -1 || next_grid.parent_y == -1){
        std::cout<<"cannot make global path"<<std::endl;
        std::cout<<"global_path_planner is shutting down"<<std::endl;
        std::exit(1);
    }
    else searching_grid_.set_parameter(next_grid.parent_x,next_grid.parent_y);
}

void AStarPlanner::check_goal()
{
    if(open_list_[goal_grid_.x][goal_grid_.y].f < WALL_COST_) is_goal_ = true;
}

void AStarPlanner::trace_dealer(nav_msgs::Path& waypoint_path)
{
    is_start_ = false;
    add_path_point(waypoint_path,goal_grid_.x,goal_grid_.y);
    tracing_grid_.set_parameter(open_list_[goal_grid_.x][goal_grid_.y].parent_x,
                                open_list_[goal_grid_.x][goal_grid_.y].parent_y);
    while(!is_start_){
        add_path_point(waypoint_path,tracing_grid_.x,tracing_grid_.y);
        Coordinate store_grid;
        store_grid.set_parameter(tracing_grid_.x,tracing_grid_.y);
        tracing_grid_.set_parameter(close_list_[store_grid.x][store_grid.y].parent_x,
                                    close_list_[store_grid.x][store_grid.y].parent_y);
        if(tracing_grid_.x == close_list_[tracing_grid_.x][tracing_grid_.y].parent_x && tracing_grid_.y == close_list_[tracing_grid_.x][tracing_grid_.y].parent_y){
            add_path_point(waypoint_path,tracing_grid_.x,tracing_grid_.y);
            is_start_ = true;
        }
    }
}

void AStarPlanner::add_path_point(nav_msgs::Path& waypoint_path,int x,int y)
{
    geometry_msgs::PoseStamped path_point;
    path_point.pose.position.x = ((double)(x - center_.x)*resolution_);
    path_point.pose.position.y = ((double)(y - center_.y)*resolution_);
    waypoint_path.poses.push_back(path_point);
}

void AStarPlanner::publish_global_path()
{
    path_pub_.publish(global_path_);
}

float AStarPlanner::calc_huristic(int x,int y)
{
    return std::sqrt(std::pow(x - goal_grid_.x,2) + std::pow(y - goal_grid_.y,2));
}

void AStarPlanner::process()
{
    ros::Rate rate(HZ_);
    while(ros::ok()){
        if(has_received_map_){
            publish_global_path();
        }
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"a_star_planner");
    AStarPlanner a_star_planner;
    a_star_planner.process();
    return 0;
}
