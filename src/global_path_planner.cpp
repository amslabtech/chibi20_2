#include "global_path_planner/global_path_planner.h"

A_Star_Planner::A_Star_Planner() :private_n("~")
{
    //parameter

    //subscriber

    //publisher

}

void A_Star_Planner::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    nav_msgs::OccupancyGrid _msg=*msg;
    //std::cout<<"map received"<<std::endl;
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

void A_Star_Planner::clean_close_list()
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
    // open_list[searching_grid.x][searching_grid.y].dealer_x=-1;
    // open_list[searching_grid.x][searching_grid.y].dealer_y=-1;
}

void A_Star_Planner::define_goal_grid()
{
    //１周するには何回かゴールを更新して１つのパスを作る必要がある。
    //全体的なスタートとゴールが一緒なためそのままやるとパスを引けない
    //ゴール設定方法を検討中
}

void A_Star_Planner::open_around()
{
    if(searching_grid.x>0)
    {
        open_list[searching_grid.x-1][searching_grid.y].g=open_list[searching_grid.x][searching_grid.y].g+move_cost[0];
        open_list[searching_grid.x-1][searching_grid.y].f=open_list[searching_grid.x-1][searching_grid.y].g+huristic(searching_grid.x-1,searching_grid.y);
        open_list[searching_grid.x-1][searching_grid.y].dealer_x=searching_grid.x;
        open_list[searching_grid.x-1][searching_grid.y].dealer_y=searching_grid.y;
        if(grid_map[searching_grid.x-1][searching_grid.y]>wall_border)
            open_list[searching_grid.x-1][searching_grid.y].f+=wall_cost;
    }

    if(searching_grid.y<column-1)
    {
        open_list[searching_grid.x][searching_grid.y+1].g=open_list[searching_grid.x][searching_grid.y].g+move_cost[1];
        open_list[searching_grid.x][searching_grid.y+1].f=open_list[searching_grid.x][searching_grid.y+1].g+huristic(searching_grid,searching_grid.y+1);
        open_list[searching_grid.x][searching_grid.y+1].dealer_x=searching_grid.x;
        open_list[searching_grid.x][searching_grid.y+1].dealer_y=searching_grid.y;
        if(grid_map[searching_grid.x][searching_grid.y+1]>wall_border)
            open_list[searching_grid.x][searching_grid.y+1].f+=wall_cost;
    }

    if(searching_grid.x<row-1)
    {
        open_list[searching_grid.x+1][searching_grid.y].g=open_list[searching_grid.x][searching_grid.y].g+move_cost[2];
        open_list[searching_grid.x+1][searching_grid.y].f=open_list[searching_grid.x+1][searching_grid.y].g+huristic(searching_grid.x+1,searching_grid.y);
        open_list[searching_grid.x+1][searching_grid.y].dealer_x=searching_grid.x;
        open_list[searching_grid.x+1][searching_grid.y].dealer_y=searching_grid.y;
        if(grid_map[searching_grid.x+1][searching_grid.y]>wall_border)
            open_list[searching_grid.x+1][searching_grid.y].f+=wall_cost;
    }

    if(searching_grid.y>0)
    {
        open_list[searching_grid.x][searching_grid.y-1].g=open_list[searching_grid.x][searching_grid.y].g+move_cost[3];
        open_list[searching_grid.x][searching_grid.y-1].f=open_list[searching_grid.x][searching_grid.y-1].g+huristic(searching_grid.x,searching_grid.y-1);
        open_list[searching_grid.x][searching_grid.y-1].dealer_x=searching_grid.x;
        open_list[searching_grid.x][searching_grid.y-1].dealer_y=searching_grid.y;
        if(grid_map[searching_grid.x][searching_grid.y-1]>wall_border)
            open_list[searching_grid.x][searching_grid.y-1].f+=wall_cost;
    }
}

void A_Star_Planner::close_delar()
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
        std::cout<<"cannot make gloabal path"<<std::endl;
        exit;
    }
    else
    {
        searching_grid.x=next_grid.dealer_x;
        searching_grid.y=next_grid.dealer_y;
    }
}



void A_Star_Planner::path_creator()
{
    clean_lists();
    define_starting_grid();
    define_goal_grid();

    open_around();
    close_dealer();
    check_goal();
    update_searching_grid();




}

int main (int argc, char **argv)
{
  ros::init(argc,argv,"global_path_planner");
  A_Star_Planner a_star_planner;
  a_star_planner.process();
  return 0;
}









