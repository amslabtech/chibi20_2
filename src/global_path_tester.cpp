#include "global_path_planner/global_path_planner.h"

A_Star_Planner::A_Star_Planner() :private_n("~")
{
    //parameter
    private_n.param("row",row,{1000});
    private_n.param("column",column,{1000});
    private_n.param("scale",scale,{20});
    private_n.param("Hz",Hz,{1});
    private_n.param("wall_border",wall_border,{50});
    private_n.param("wall_cost",wall_cost,{100000000});
    private_n.param("map_received",map_received,{false});
    private_n.param("reached_goal",reached_goal,{false});
    private_n.param("reached_start",reached_start,{false});
    private_n.param("proto_g",proto_g,{100000000});
    private_n.param("proto_f",proto_f,{100000000});

    // //subscriber
    sub_map=n.subscribe("map",10,&A_Star_Planner::map_callback,this);
    // sub_pose=n.subscribe("chibi20_2/localizer",1,&A_Star_Planner::pose_callback,this);
    //
    // //publisher
    pub_path=n.advertise<nav_msgs::Path>("global_path",1);
    pub_open_grid=n.advertise<geometry_msgs::PointStamped>("searching_grid",1);


}

void A_Star_Planner::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("map_received");
    if(map_received)
        return;
    prior_map=*msg;
    row=prior_map.info.height;
    column=prior_map.info.width;
    grid_map.resize(row,std::vector<int>(column,-1));
    open_list.resize(row,std::vector<Costs>(column));
    close_list.resize(row,std::vector<Costs>(column));
    std::cout<<"map size"<<grid_map.size()<<"*"<<grid_map.at(0).size()<<std::endl;
    for(int i=0;i<column;i++)
    {
        for(int j=0;j<row;j++)
        {
            grid_map[j][i]=prior_map.data[i*column+j];
        }
    }
    adjust.x=row/2;
    adjust.y=column/2;
    //###use in test only
    // searching_grid.x=1000+adjust.x;
    // searching_grid.y=700+adjust.y;
    // std::cout<<searching_grid.x<<","<<searching_grid.y<<std::endl;
    //###

    global_path.header.frame_id="map";
    map_received= true;
}

// void A_Star_Planner::pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
// {
//      current_pose=*msg;
// }
//
void A_Star_Planner::clean_lists()
{
    // ROS_INFO("starting clean_lists");
    for(int i=0;i<row;i++)
    {
        for(int j=0;j<column;j++)
        {
            close_list[i][j].g=wall_cost;
            close_list[i][j].f=wall_cost;
            close_list[i][j].dealer_x=-1;
            close_list[i][j].dealer_y=-1;
            open_list[i][j].g=wall_cost;
            open_list[i][j].f=wall_cost;
            open_list[i][j].dealer_x=-1;
            open_list[i][j].dealer_y=-1;
        }
        global_path.poses.resize(0);
    }
    // ROS_INFO("completed clean_lists");
}

float A_Star_Planner::huristic(const int& x,const int& y)
{
    return sqrt(pow(x-goal_grid.x,2)+pow(y-goal_grid.y,2));
    // return fabs(x-goal_grid.x)+fabs(y-goal_grid.y);
}

void A_Star_Planner::define_starting_grid()
{
    // ROS_INFO("starting define_starting_grid");
    // searching_grid.x=(int)current_pose.x;
    // searching_grid.y=(int)current_pose.y;
    searching_grid.x=3000;
    searching_grid.y=2700;
    std::cout<<searching_grid.x<<","<<searching_grid.y<<std::endl;
    reached_goal=false;
    // ROS_INFO("completed define_starting_grid");
}
//
void A_Star_Planner::define_goal_grid()
{
    // ROS_INFO("starting define_goal_grid");
    //###use in test only
    goal_grid.x=3150;
    goal_grid.y=3000;
    //###
    //１周するには何回かゴールを更新して１つのパスを作る必要がある。
    //全体的なスタートとゴールが一緒なためそのままやるとパスを引けない
    //ゴール設定方法を検討中
    open_list[searching_grid.x][searching_grid.y].f=huristic(searching_grid.x,searching_grid.y);
    open_list[searching_grid.x][searching_grid.y].g=0.0;
    open_list[searching_grid.x][searching_grid.y].dealer_x=searching_grid.x;
    open_list[searching_grid.x][searching_grid.y].dealer_y=searching_grid.y;
    // std::cout<<open_list[searching_grid.x][searching_grid.y].f<<std::endl;
    // ROS_INFO("completed define_goal_grid");
}
//
int A_Star_Planner::grid_patern(const int& x,const int& y)
{
    if(open_list[x][y].f<wall_cost)
        return 1;//This grid is in the open_list
    else
    {
        if(close_list[x][y].f<wall_cost)
            return 2;//This grid is in the close_list
        else
            return 0;//This grid isn't in any lists
    }
}

void A_Star_Planner::update_open_list(const int& x,const int& y)
{
    if(grid_map[searching_grid.x+x][searching_grid.y+y]>wall_border||grid_map[searching_grid.x+x][searching_grid.y+y]<0)
        return;
    open_list[searching_grid.x+x][searching_grid.y+y].g=proto_g;
    open_list[searching_grid.x+x][searching_grid.y+y].f=proto_f;
    open_list[searching_grid.x+x][searching_grid.y+y].dealer_x=searching_grid.x;
    open_list[searching_grid.x+x][searching_grid.y+y].dealer_y=searching_grid.y;
}

void A_Star_Planner::clean_close_list(const int& x,const int& y)
{
    close_list[searching_grid.x+x][searching_grid.y+y].g=wall_cost;
    close_list[searching_grid.x+x][searching_grid.y+y].f=wall_cost;
    close_list[searching_grid.x+x][searching_grid.y+y].dealer_x=-1;
    close_list[searching_grid.x+x][searching_grid.y+y].dealer_y=-1;
}

void A_Star_Planner::open_grid(const int& x,const int& y,const int& direction)
{
    //x=-1,y=0,direction=0
    proto_g=open_list[searching_grid.x][searching_grid.y].g+move_cost[direction];
    proto_f=proto_g+huristic(searching_grid.x+x,searching_grid.y+y);
    switch(grid_patern(searching_grid.x+x,searching_grid.y+y))
    {
     case(0):
         update_open_list(x,y);
         break;
     case(1):
         if(proto_f<open_list[searching_grid.x+x][searching_grid.y+y].f)
             update_open_list(x,y);
         break;
     case(2):
         if(proto_f<close_list[searching_grid.x+x][searching_grid.y+y].f)
         {
             update_open_list(x,y);
             clean_close_list(x,y);
         }
         break;
     default: break;
    }
}


void A_Star_Planner::open_around()
{
    // ROS_INFO("starting open_around");
    // if(searching_grid.x>0)
        open_grid(-1,0,0);
    // if(searching_grid.x>0&&searching_grid.y<column-1)
        open_grid(-1,1,1);
    // if(searching_grid.y<column-1)
        open_grid(0,1,2);
    // if(searching_grid.y<column-1&&searching_grid.x<row-1)
        open_grid(1,1,3);
    // if(searching_grid.x<row-1)
        open_grid(1,0,4);
    // if(searching_grid.x<row-1&&searching_grid.y>0)
        open_grid(1,-1,5);
    // if(searching_grid.y>0)
        open_grid(0,-1,6);
    // if(searching_grid.y>0&&searching_grid.x>0)
        open_grid(-1,-1,7);
}

void A_Star_Planner::update_close_list()
{
    // ROS_INFO("starting update_close_list");
    close_list[searching_grid.x][searching_grid.y].g=open_list[searching_grid.x][searching_grid.y].g;
    close_list[searching_grid.x][searching_grid.y].f=open_list[searching_grid.x][searching_grid.y].f;
    close_list[searching_grid.x][searching_grid.y].dealer_x=open_list[searching_grid.x][searching_grid.y].dealer_x;
    close_list[searching_grid.x][searching_grid.y].dealer_y=open_list[searching_grid.x][searching_grid.y].dealer_y;
    open_list[searching_grid.x][searching_grid.y].g=wall_cost;
    open_list[searching_grid.x][searching_grid.y].f=wall_cost;
    open_list[searching_grid.x][searching_grid.y].dealer_x=-1;
    open_list[searching_grid.x][searching_grid.y].dealer_y=-1;
    // ROS_INFO("completed update_close_list");
}

void A_Star_Planner::check_goal()
{
    // ROS_INFO("starting check_goal");
    if(open_list[goal_grid.x][goal_grid.y].f<wall_cost)
        reached_goal=true;
    // ROS_INFO("completed check_goal");
}

void A_Star_Planner::update_searching_grid()
{
    // ROS_INFO("starting update_searching_grid");
    Costs next_grid;
    next_grid.g=wall_cost;
    next_grid.f=wall_cost;
    next_grid.dealer_x=-1;
    next_grid.dealer_y=-1;
    for(int i=0;i<row;i++)
    {
        for(int j=0;j<column;j++)
        {
            if(open_list[i][j].f<next_grid.f)
            {
                next_grid.g=open_list[i][j].g;
                next_grid.f=open_list[i][j].f;
                next_grid.dealer_x=i;
                next_grid.dealer_y=j;
            }
        }
    }
    if(next_grid.dealer_x==-1||next_grid.dealer_y==-1)
    {
        std::cout<<"cannot make global path"<<std::endl;
        show_open_list();
        show_close_list();
        std::exit(1);
    }
    else
    {
        searching_grid.x=next_grid.dealer_x;
        searching_grid.y=next_grid.dealer_y;
    }
    // ROS_INFO("completed update_searching_grid");
}

void A_Star_Planner::add_path_point(const int& x,const int& y)
{
    geometry_msgs::PoseStamped path_point;
    path_point.pose.position.x=((double)(x-adjust.x)/scale);
    path_point.pose.position.y=((double)(y-adjust.y)/scale);
    global_path.poses.push_back(path_point);
}

void A_Star_Planner::trace_dealer()
{
    ROS_INFO("starting trace_dealer");
    reached_start=false;
    ROS_INFO("0");
    add_path_point(goal_grid.x,goal_grid.y);
    // add_path_point(open_list[goal_grid.x][goal_grid.y].dealer_x,open_list[goal_grid.x][goal_grid.y].dealer_y);
    tracing_grid.x=open_list[goal_grid.x][goal_grid.y].dealer_x;
    tracing_grid.y=open_list[goal_grid.x][goal_grid.y].dealer_y;
    std::cout<<"tracing_grid:"<<tracing_grid.x<<","<<tracing_grid.y<<std::endl;
    std::cout<<"next_dealer"<<close_list[tracing_grid.x][tracing_grid.y].dealer_x<<","<<close_list[tracing_grid.x][tracing_grid.y].dealer_y<<std::endl;
    ROS_INFO("1");
    while(!reached_start)
   {
       add_path_point(tracing_grid.x,tracing_grid.y);
       Coordinate reminder;
       reminder.x=tracing_grid.x;
       reminder.y=tracing_grid.y;
       tracing_grid.x=close_list[reminder.x][reminder.y].dealer_x;
       tracing_grid.y=close_list[reminder.x][reminder.y].dealer_y;
       if(tracing_grid.x==close_list[tracing_grid.x][tracing_grid.y].dealer_x&&tracing_grid.y==close_list[tracing_grid.x][tracing_grid.y].dealer_y)
           reached_start=true;
        std::cout<<tracing_grid.x<<","<<tracing_grid.y<<std::endl;
   }
   ROS_INFO("completed trace_dealer");
}

void A_Star_Planner::show_open_list()
{
    std::cout<<"open_list"<<std::endl;
    for(int i=0;i<row;i++)
    {
        for(int j=0;j<column;j++)
        {
            if(open_list[i][j].f<wall_cost)
            {
                std::cout<<i<<","<<j<<std::endl;
                std::cout<<"f="<<open_list[i][j].f<<std::endl;
                std::cout<<"dealer "<<open_list[i][j].dealer_x<<","<<open_list[i][j].dealer_y<<std::endl<<std::endl;
            }
        }
    }
}

void A_Star_Planner::show_close_list()
{
    std::cout<<"close_list"<<std::endl;
    for(int i=0;i<row;i++)
    {
        for(int j=0;j<column;j++)
        {
            if(close_list[i][j].f<wall_cost)
            {
                std::cout<<i<<","<<j<<std::endl;
                std::cout<<"f="<<close_list[i][j].f<<std::endl;
                std::cout<<"dealer "<<close_list[i][j].dealer_x<<","<<close_list[i][j].dealer_y<<std::endl<<std::endl;
            }
        }
    }
}

void A_Star_Planner::path_creator()
{
    // ROS_INFO("starting path_creator");
    clean_lists();
    define_starting_grid();
    define_goal_grid();
        std::cout<<"goal_grid"<<goal_grid.x<<","<<goal_grid.y<<std::endl;
    while(!reached_goal)
    {
        // std::cout<<"searching_grid"<<searching_grid.x<<","<<searching_grid.y<<std::endl;
        //###use test only
        geometry_msgs::PointStamped searching_grid_pose;
        searching_grid_pose.point.x=((double)(searching_grid.x-adjust.x))/scale;
        searching_grid_pose.point.y=((double)(searching_grid.y-adjust.y))/scale;
        searching_grid_pose.header.frame_id="map";
        pub_open_grid.publish(searching_grid_pose);
        //###
        open_around();
        update_close_list();
        update_searching_grid();
        check_goal();
    }
    // show_open_list();
    // show_close_list();
    trace_dealer();
    // ROS_INFO("completed path_creator");
}

void A_Star_Planner::process()
{
    ros::Rate loop_rate(Hz);
    while(ros::ok())
    {
        if(map_received)
        {
             path_creator();
             // int i=0;
             // while(1)
             // {
             //     if(!(global_path.poses[i].pose.position.x>0))
             //         break;
             //     std::cout<<global_path.poses[i].pose.position.x<<","<<global_path.poses[i].pose.position.y<<std::endl;
             //     i++;
             // }
             pub_path.publish(global_path);
                // map_received=false;
        }
           std::cout<<"test now"<<std::endl;
           std::cout<<"-----"<<std::endl;;
           map_received=false;
           ros::spinOnce();
           loop_rate.sleep();
    }
}

int main (int argc, char **argv)
{
  ros::init(argc,argv,"global_path_planner");
  std::cout<<"test now"<<std::endl;
  A_Star_Planner a_star_planner;
  std::cout<<"make a_star_planner"<<std::endl;
  a_star_planner.process();
  return 0;
}
