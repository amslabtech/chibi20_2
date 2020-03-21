#include "map_maneger/map_maneger.h"

Map_Maneger::Map_Maneger():private_nh("~")
{
   // parameter
   private_nh.param("hz",hz,{1});
   private_nh.param("row",row,{2000});
   private_nh.param("column",column,{2000});
   private_nh.param("degree",degree,{-15});
   // subscriber
   sub_map = nh.subscribe("map",10,&Map_Maneger::map_callback,this);
   sub_blocked_map = nh.subscribe("blocked_map",10,&Map_Maneger::blocked_map_callback,this);
   // publisher
   pub_fixed_map = nh.advertise<nav_msgs::OccupancyGrid>("fixed_map",1);
   //
}

void Map_Maneger::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("recieved_original_map");
    original_map  =  *msg;
    set_grid_map_parametor();
    fix_map();
    pub_fixed_map.publish(fixed_map);
    ROS_INFO("sended_fixed_map");
}

void Map_Maneger::blocked_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("recieved_blocked_map");
    fixed_map = *msg;
    pub_fixed_map.publish(fixed_map);
    ROS_INFO("sended_fixed_map");
}

void Map_Maneger::set_grid_map_parametor()
{
    row = original_map.info.height;
    column = original_map.info.width;
    grid_map.resize(row,std::vector<int>(column,-1));
    std::cout<<"map size"<<grid_map.size()<<"*"<<grid_map.at(0).size()<<std::endl;
    for(int i = 0;i < column; i++)
    {
        for(int j = 0;j < row;j++)
        {
            grid_map[j][i] = original_map.data[i*column+j];
        }
    }
    adjust.x = row/2;
    adjust.y = column/2;
}

void Map_Maneger::move_map_parallel()
{
    std::vector<std::vector<int>> moved_map;
    moved_map.resize(row,std::vector<int>(column,-1));
    Coordinate move_length;
    move_length.x = 975;
    move_length.y = 690;
    for(int i = 0;i<row;i++)
    {
        for(int j = 0;j<column;j++)
        {
            if(grid_map[i][j] != -1)
            {
                 moved_map[i-move_length.x][j-move_length.y] = grid_map[i][j];
            }
        }
    }
    for(int i = 0;i < row;i++)
    {
        for(int j = 0; j < column ;j++)
        {
            grid_map[i][j] = moved_map[i][j];
        }
    }
}

void Map_Maneger::turn_map()
{
    std::vector<std::vector<int>>rotated_map;
    rotated_map.resize(row,std::vector<int>(column,-1));
    double theta = degree*M_PI/180;
    for(int i = 0;i<row;i++)
    {
        for(int j = 0;j<column;j++)
        {
            if(((int)((i-adjust.x)*cos(theta)-(j-adjust.y)*sin(theta)+adjust.x)) >=  0&& ((int)((i-adjust.x)*sin(theta)+(j-adjust.y)*cos(theta)+adjust.y)) >= 0 && ((int)((i-adjust.x)*cos(theta)-(j-adjust.y)*sin(theta)+adjust.x)) < row && ((int)((i-adjust.x)*sin(theta)+(j-adjust.y)*cos(theta)+adjust.y)) < column)
            {
                // std::cout<<(int)((i-adjust.x)*cos(theta)-(j-adjust.y)*sin(theta)+adjust.x)<<","<<(int)((i-adjust.x)*sin(theta)+(j-adjust.y)*cos(theta)+adjust.y)<<std::endl;
                rotated_map[i][j] = grid_map[(int)((i-adjust.x)*cos(theta)-(j-adjust.y)*sin(theta)+adjust.x)][(int)((i-adjust.x)*sin(theta)+(j-adjust.y)*cos(theta)+adjust.y)];
            }
        }
    }
    for(int i = 0;i < row;i++)
    {
        for(int j = 0; j < column ;j++)
        {
            grid_map[i][j] = rotated_map[i][j];
        }
    }
}

void Map_Maneger::fix_map()
{
    move_map_parallel();
    turn_map();
    fixed_map.info = original_map.info;
    for(int i = 0;i<column;i++)
    {
        for(int j = 0;j<row;j++)
        {
            fixed_map.data.push_back(grid_map[j][i]);
        }
    }
}

void Map_Maneger::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {

    ros::spinOnce();
    loop_rate.sleep();
    }
}


int main (int argc, char **argv)
{
  ros::init(argc,argv,"map_maneger");
  Map_Maneger map_maneger;
  std::cout<<"map_maneger has started"<<std::endl;
  map_maneger.process();
  return 0;
}
