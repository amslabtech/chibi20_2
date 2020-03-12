#include "global_path_planner/global_path_planner.h"

A_Star_Planner::A_Star_Planner() :private_n("~")
{
    //parameter
    private_n.param("Hz",Hz,{1});
    private_n.param("wall_border",wall_border,{50});
    private_n.param("wall_cost",wall_cost,{100000000});
    // private_n.param("move_cost",move_cost,{1,1,1,1});
    private_n.param("map_received",map_received,{false});
    private_n.param("reached_goal",reached_goal,{false});
    private_n.param("reached_start",reached_start,{false});

    //subscriber
    sub_map=n.subscribe("chibi20_2/map",1,&A_Star_Planner::map_callback,this);
    sub_pose=n.subscribe("chibi20_2/localizer",1,&A_Star_Planner::pose_callback,this);

    //publisher
    pub_path=n.advertise<nav_msgs::Path>("/chibi20_2/global_path_planner",1);


}

void A_Star_Planner::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    nav_msgs::OccupancyGrid _msg=*msg;
    //std::cout<<"map received"<<std::endl;
    map_received=true;
    for(int i=0;i<row;i++)
    {
        for(int j=0;j<column;j++)
        {
            grid_map[i][j]= _msg.data[i*row+j];
        }
    }

    //global_path.header.frame_id="map";

}

void A_Star_Planner::pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
     current_pose=*msg;
}

void A_Star_Planner::clean_lists()
{
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
    }
}

float A_Star_Planner::huristic(const int& x,const int& y)
{
    return sqrt(pow(x-goal_grid.x,2)+pow(y-goal_grid.y,2));
}

void A_Star_Planner::define_starting_grid()
{
    searching_grid.x=(int)current_pose.x;
    searching_grid.y=(int)current_pose.y;
    open_list[searching_grid.x][searching_grid.y].f=huristic(searching_grid.x,searching_grid.y);
    open_list[searching_grid.x][searching_grid.y].g=0.0;
    open_list[searching_grid.x][searching_grid.y].dealer_x=searching_grid.x;
    open_list[searching_grid.x][searching_grid.y].dealer_y=searching_grid.y;
}

void A_Star_Planner::define_goal_grid()
{
    //１周するには何回かゴールを更新して１つのパスを作る必要がある。
    //全体的なスタートとゴールが一緒なためそのままやるとパスを引けない
    //ゴール設定方法を検討中
}

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


void A_Star_Planner::open_around()
{
    if(searching_grid.x>0)
    {
        proto_g=open_list[searching_grid.x][searching_grid.y].g+move_cost[0];
        proto_f=proto_g+huristic(searching_grid.x-1,searching_grid.y);
        switch(grid_patern(searching_grid.x-1,searching_grid.y))
        {
         case(0):
             open_list[searching_grid.x-1][searching_grid.y].g=proto_g;
             open_list[searching_grid.x-1][searching_grid.y].f=proto_f;
             open_list[searching_grid.x-1][searching_grid.y].dealer_x=searching_grid.x;
             open_list[searching_grid.x-1][searching_grid.y].dealer_y=searching_grid.y;
             if(grid_map[searching_grid.x-1][searching_grid.y]>wall_border)
                 open_list[searching_grid.x-1][searching_grid.y].f+=wall_cost;
             break;
         case(1):
             if(proto_f<open_list[searching_grid.x-1][searching_grid.y].f)
             {
                 open_list[searching_grid.x-1][searching_grid.y].g=proto_g;
                 open_list[searching_grid.x-1][searching_grid.y].g=proto_f;
                 open_list[searching_grid.x-1][searching_grid.y].dealer_x=searching_grid.x;
                 open_list[searching_grid.x-1][searching_grid.y].dealer_y=searching_grid.y;
                 if(grid_map[searching_grid.x-1][searching_grid.y]>wall_border)
                     open_list[searching_grid.x-1][searching_grid.y].f+=wall_cost;
             }
             break;
         case(2):
             if(proto_f<close_list[searching_grid.x-1][searching_grid.y].f)
             {
                 open_list[searching_grid.x-1][searching_grid.y].g=proto_g;
                 open_list[searching_grid.x-1][searching_grid.y].f=proto_f;
                 open_list[searching_grid.x-1][searching_grid.y].dealer_x=searching_grid.x;
                 open_list[searching_grid.x-1][searching_grid.y].dealer_y=searching_grid.y;
                 if(grid_map[searching_grid.x-1][searching_grid.y]>wall_border)
                     open_list[searching_grid.x-1][searching_grid.y].f+=wall_cost;
                 close_list[searching_grid.x-1][searching_grid.y].g=wall_cost;
                 close_list[searching_grid.x-1][searching_grid.y].f=wall_cost;
                 close_list[searching_grid.x-1][searching_grid.y].dealer_x=-1;
                 close_list[searching_grid.x-1][searching_grid.y].dealer_y=-1;
             }
             break;
         default: break;
        }
    }

    if(searching_grid.y<column-1)
    {
        proto_g=open_list[searching_grid.x][searching_grid.y].g+move_cost[1];
        proto_f=proto_g+huristic(searching_grid.x,searching_grid.y+1);
        switch(grid_patern(searching_grid.x,searching_grid.y+1))
        {
         case(0):
             open_list[searching_grid.x][searching_grid.y+1].g=proto_g;
             open_list[searching_grid.x][searching_grid.y+1].f=proto_f;
             open_list[searching_grid.x][searching_grid.y+1].dealer_x=searching_grid.x;
             open_list[searching_grid.x][searching_grid.y+1].dealer_y=searching_grid.y;
             if(grid_map[searching_grid.x][searching_grid.y+1]>wall_border)
                 open_list[searching_grid.x][searching_grid.y+1].f+=wall_cost;
             break;
         case(1):
             if(proto_f<open_list[searching_grid.x][searching_grid.y+1].f)
             {
                 open_list[searching_grid.x][searching_grid.y+1].g=proto_g;
                 open_list[searching_grid.x][searching_grid.y+1].g=proto_f;
                 open_list[searching_grid.x][searching_grid.y+1].dealer_x=searching_grid.x;
                 open_list[searching_grid.x][searching_grid.y+1].dealer_y=searching_grid.y;
                 if(grid_map[searching_grid.x][searching_grid.y+1]>wall_border)
                     open_list[searching_grid.x][searching_grid.y+1].f+=wall_cost;
             }
             break;
         case(2):
             if(proto_f<close_list[searching_grid.x][searching_grid.y+1].f)
             {
                 open_list[searching_grid.x][searching_grid.y+1].g=proto_g;
                 open_list[searching_grid.x][searching_grid.y+1].f=proto_f;
                 open_list[searching_grid.x][searching_grid.y+1].dealer_x=searching_grid.x;
                 open_list[searching_grid.x][searching_grid.y+1].dealer_y=searching_grid.y;
                 if(grid_map[searching_grid.x][searching_grid.y+1]>wall_border)
                     open_list[searching_grid.x][searching_grid.y+1].f+=wall_cost;
                 close_list[searching_grid.x][searching_grid.y+1].g=wall_cost;
                 close_list[searching_grid.x][searching_grid.y+1].f=wall_cost;
                 close_list[searching_grid.x][searching_grid.y+1].dealer_x=-1;
                 close_list[searching_grid.x][searching_grid.y+1].dealer_y=-1;
             }
             break;
         default: break;
        }
    }
    if(searching_grid.x<column-1)
    {
        proto_g=open_list[searching_grid.x][searching_grid.y].g+move_cost[2];
        proto_f=proto_g+huristic(searching_grid.x+1,searching_grid.y);
        switch(grid_patern(searching_grid.x+1,searching_grid.y))
        {
         case(0):
             open_list[searching_grid.x+1][searching_grid.y].g=proto_g;
             open_list[searching_grid.x+1][searching_grid.y].f=proto_f;
             open_list[searching_grid.x+1][searching_grid.y].dealer_x=searching_grid.x;
             open_list[searching_grid.x+1][searching_grid.y].dealer_y=searching_grid.y;
             if(grid_map[searching_grid.x+1][searching_grid.y]>wall_border)
                 open_list[searching_grid.x+1][searching_grid.y].f+=wall_cost;
             break;
         case(1):
             if(proto_f<open_list[searching_grid.x+1][searching_grid.y].f)
             {
                 open_list[searching_grid.x+1][searching_grid.y].g=proto_g;
                 open_list[searching_grid.x+1][searching_grid.y].g=proto_f;
                 open_list[searching_grid.x+1][searching_grid.y].dealer_x=searching_grid.x;
                 open_list[searching_grid.x+1][searching_grid.y].dealer_y=searching_grid.y;
                 if(grid_map[searching_grid.x+1][searching_grid.y]>wall_border)
                     open_list[searching_grid.x+1][searching_grid.y].f+=wall_cost;
             }
             break;
         case(2):
             if(proto_f<close_list[searching_grid.x+1][searching_grid.y].f)
             {
                 open_list[searching_grid.x+1][searching_grid.y].g=proto_g;
                 open_list[searching_grid.x+1][searching_grid.y].f=proto_f;
                 open_list[searching_grid.x+1][searching_grid.y].dealer_x=searching_grid.x;
                 open_list[searching_grid.x+1][searching_grid.y].dealer_y=searching_grid.y;
                 if(grid_map[searching_grid.x+1][searching_grid.y]>wall_border)
                     open_list[searching_grid.x+1][searching_grid.y].f+=wall_cost;
                 close_list[searching_grid.x+1][searching_grid.y].g=wall_cost;
                 close_list[searching_grid.x+1][searching_grid.y].f=wall_cost;
                 close_list[searching_grid.x+1][searching_grid.y].dealer_x=-1;
                 close_list[searching_grid.x+1][searching_grid.y].dealer_y=-1;
             }
             break;
         default: break;
        }
    }

    if(searching_grid.y>0)
    {
        proto_g=open_list[searching_grid.x][searching_grid.y].g+move_cost[3];
        proto_f=proto_g+huristic(searching_grid.x,searching_grid.y+1);
        switch(grid_patern(searching_grid.x,searching_grid.y+1))
        {
         case(0):
             open_list[searching_grid.x][searching_grid.y-1].g=proto_g;
             open_list[searching_grid.x][searching_grid.y-1].f=proto_f;
             open_list[searching_grid.x][searching_grid.y-1].dealer_x=searching_grid.x;
             open_list[searching_grid.x][searching_grid.y-1].dealer_y=searching_grid.y;
             if(grid_map[searching_grid.x][searching_grid.y-1]>wall_border)
                 open_list[searching_grid.x][searching_grid.y-1].f+=wall_cost;
             break;
         case(1):
             if(proto_f<open_list[searching_grid.x][searching_grid.y-1].f)
             {
                 open_list[searching_grid.x][searching_grid.y-1].g=proto_g;
                 open_list[searching_grid.x][searching_grid.y-1].g=proto_f;
                 open_list[searching_grid.x][searching_grid.y-1].dealer_x=searching_grid.x;
                 open_list[searching_grid.x][searching_grid.y-1].dealer_y=searching_grid.y;
                 if(grid_map[searching_grid.x][searching_grid.y-1]>wall_border)
                     open_list[searching_grid.x][searching_grid.y-1].f+=wall_cost;
             }
             break;
         case(2):
             if(proto_f<close_list[searching_grid.x][searching_grid.y-1].f)
             {
                 open_list[searching_grid.x][searching_grid.y-1].g=proto_g;
                 open_list[searching_grid.x][searching_grid.y-1].f=proto_f;
                 open_list[searching_grid.x][searching_grid.y-1].dealer_x=searching_grid.x;
                 open_list[searching_grid.x][searching_grid.y-1].dealer_y=searching_grid.y;
                 if(grid_map[searching_grid.x][searching_grid.y-1]>wall_border)
                     open_list[searching_grid.x][searching_grid.y-1].f+=wall_cost;
                 close_list[searching_grid.x][searching_grid.y-1].g=wall_cost;
                 close_list[searching_grid.x][searching_grid.y-1].f=wall_cost;
                 close_list[searching_grid.x][searching_grid.y-1].dealer_x=-1;
                 close_list[searching_grid.x][searching_grid.y-1].dealer_y=-1;
             }
             break;
         default: break;
        }
    }
}

void A_Star_Planner::update_close_list()
{
    close_list[searching_grid.x][searching_grid.y].g=open_list[searching_grid.x][searching_grid.y].g;
    close_list[searching_grid.x][searching_grid.y].f=open_list[searching_grid.x][searching_grid.y].f;
    close_list[searching_grid.x][searching_grid.y].dealer_x=open_list[searching_grid.x][searching_grid.y].dealer_x;
    close_list[searching_grid.x][searching_grid.y].dealer_y=open_list[searching_grid.x][searching_grid.y].dealer_y;
    open_list[searching_grid.x][searching_grid.y].g=wall_cost;
    open_list[searching_grid.x][searching_grid.y].f=wall_cost;
    open_list[searching_grid.x][searching_grid.y].dealer_x=-1;
    open_list[searching_grid.x][searching_grid.y].dealer_y=-1;
}

void A_Star_Planner::check_goal()
{
    if(open_list[goal_grid.x][goal_grid.y].f<wall_cost)
        reached_goal=true;
}

void A_Star_Planner::update_searching_grid()
{
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
        std::exit(0);
    }
    else
    {
        searching_grid.x=next_grid.dealer_x;
        searching_grid.y=next_grid.dealer_y;
    }
}

void A_Star_Planner::trace_dealer()
{
    reached_start=false;
    global_path.poses[0].pose.position.x=goal_grid.x;
    global_path.poses[0].pose.position.x=goal_grid.y;
    global_path.poses[1].pose.position.x=open_list[goal_grid.x][goal_grid.y].dealer_x;
    global_path.poses[1].pose.position.y=open_list[goal_grid.x][goal_grid.y].dealer_y;
    tracing_grid.x=goal_grid.x;
    tracing_grid.y=goal_grid.y;
    int i=2;
    while(!reached_start)
   {
       global_path.poses[i].pose.position.x=close_list[tracing_grid.x][tracing_grid.y].dealer_x;
       global_path.poses[i].pose.position.y=close_list[tracing_grid.x][tracing_grid.y].dealer_y;
       tracing_grid.x=close_list[tracing_grid.x][tracing_grid.y].dealer_x;
       tracing_grid.y=close_list[tracing_grid.x][tracing_grid.y].dealer_y;
        if(tracing_grid.x==close_list[tracing_grid.x][tracing_grid.y].dealer_x)
            reached_start=true;
   }
}

void A_Star_Planner::path_creator()
{
    clean_lists();
    define_starting_grid();
    define_goal_grid();
    while(!reached_goal)
    {
        open_around();
        update_close_list();
        update_searching_grid();
        check_goal();
    }
    trace_dealer();



}

void A_Star_Planner::process()
{
    ros::Rate loop_rate(Hz);
    if(map_received)
    {
        path_creator();
        pub_path.publish(global_path);
        map_received=false;
    }
    loop_rate.sleep();
}

int main (int argc, char **argv)
{
  ros::init(argc,argv,"global_path_planner");
  A_Star_Planner a_star_planner;
  a_star_planner.process();
  return 0;
}









