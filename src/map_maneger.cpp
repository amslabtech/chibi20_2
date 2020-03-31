#include "map_maneger/map_maneger.h"

Map_Maneger::Map_Maneger():private_nh("~")
{
   // parameter
   private_nh.param("hz",hz,{1});
   private_nh.param("row",row,{2000});
   private_nh.param("column",column,{2000});
   private_nh.param("degree",degree,{165});
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
    repaint_map();
    fixed_map.info = original_map.info;
    for(int i = 0;i<column;i++)
    {
        for(int j = 0;j<row;j++)
        {
            fixed_map.data.push_back(grid_map[j][i]);
        }
    }
}

void Map_Maneger::repaint_map()
{
    int start_point_x = 1880;
    int start_point_y = 2308;
    int end_point_x = 2124;
    int end_point_y = 2348;//上の水平通路の定義
    for(int i = start_point_x+1; i < end_point_x; i++)
    {
        for(int j = start_point_y+1; j < end_point_y; j++) grid_map[i][j] = 0;
    }
    int x2 = 1960;
    int x3 = 2047;
    int x4 = 2100;
    int x5 = 2117;
    int y2 = 2226;
    int y3 = 2290;
    int y4 = 2441;
    for(int i = start_point_x; i < x2; i++) grid_map[i][end_point_y] =100;//同通路の左上壁
    for(int i = x3; i < x4; i++) grid_map[i][end_point_y] =100;//右上壁
    for(int i = x5; i <= end_point_x; i++) grid_map[i][end_point_y] =100;//右上壁
    for(int i = start_point_x; i < x2; i++) grid_map[i][start_point_y] =100;//同通路の左下壁
    for(int i = x3; i <= end_point_x; i++) grid_map[i][start_point_y] = 100;//右下壁
    for(int j = y2; j <= start_point_y; j++) grid_map[start_point_x][j] = 100;//左垂直通路右壁
    for(int j = y2; j <= start_point_y; j++) grid_map[x2][j] = 100;//中央垂直通路左壁
    for(int j = y2; j <= start_point_y; j++) grid_map[x3][j] = 100;//中央垂直通路右壁
    for(int i = x2+1; i < x3; i++)
    {
        for(int j = end_point_y; j < y4; j++) grid_map[i][j] = 0;//上玄関前
    }
    for(int i = start_point_x+1; i < x2; i++)
    {
        for(int j = y2+1; j < start_point_y; j++) grid_map[i][j] = -1;//左上穴拡張
    }
    for(int i = x3+1; i < x5; i++)
    {
        for(int j = y2+1; j < start_point_y; j++) grid_map[i][j] = -1;//右上穴拡張
    }
    int x6 = 2053;
    int x7 = 2111;
    for(int i = x6+1; i < x7; i++)
    {
        for(int j = y3+1; j < start_point_y; j++) grid_map[i][j] = 0;//エレベータ部分
    }
    for(int i = x6; i <= x7; i++) grid_map[i][y3] = 100;//エレベータ縁
    for(int i = x6+1; i < x7; i++) grid_map[i][start_point_y] = 0;//エレベータ縁
    for(int j = y3; j <= start_point_y; j++) grid_map[x6][j] = 100;//エレベータ縁
    for(int j = y3; j <= start_point_y; j++) grid_map[x7][j] = 100;//エレベータ縁
    int x8 = 1929;
    int y5 = 2317;
    int y6 = 2300;
    for(int i = x8+1; i < x2; i++)
    {
        for(int j = y6+1; j < y5; j++) grid_map[i][j] = -1;//ゴミ箱部分
    }
    for(int j = start_point_y; j <= y5; j++) grid_map[x8][j] = 100;//ゴミ箱縁
    for(int j = y6; j <= y5; j++) grid_map[x2][j] = 100;//ゴミ箱と階段の縁
    for(int i = x8; i <= x2; i++) grid_map[i][y6] =100;//ゴミ箱縁
    for(int i = x8; i <= x2; i++) grid_map[i][y5] =100;
    int x9 = 1925;
    int x10 = 1954;
    int y7 = 2276;
    int y8 = 2262;
    for(int i = x9+1; i < x2; i++)
    {
        for(int j = y7+1; j < y6; j++) grid_map[i][j] = 0;//階段
    }
    for(int i = x9+1; i < x10; i++)
    {
        for(int j = y8+1; j <= y7; j++) grid_map[i][j] = 0;//階段
    }
    for(int j = y8; j <= y6; j++) grid_map[x9][j] = 100;//階段縁
    for(int j = y8; j <= y7; j++) grid_map[x10][j] = 100;//階段縁
    for(int i = x9; i <= x2; i++) grid_map[i][y6] = 100;//階段縁
    for(int i = x10; i <= x2; i++) grid_map[i][y7] = 100;//階段縁
    for(int i = x9; i <= x10; i++) grid_map[i][y8] =100;//階段縁
    int x11 = 1840;
    int x12 = 1875;
    // int x13 = 1957;
    // int x14 = 2042;
    int x15 = 1963;
    int x16 = 2079;
    int y9 = 1524;
    int y10 = 1646;
    int y11 = 1683;
    int y12 = 2345;
    for(int i = x11; i < x12; i++)
    {
        for(int j = y10; j < y12; j++) grid_map[i][j] = 0;//左垂直通路
    }
    for(int i = x11; i < x16+8; i++)
    {
        for(int j = y10; j < y11; j++) grid_map[i][j] = 0;//下水平通路
    }
    for(int i = x15; i < x16; i++)
    {
        for(int j = y9; j < y10; j++) grid_map[i][j] = 0;//下玄関
    }
    int x17 = 2132;
    int x18 = 2159;
    int y13 = 1633;
    int y14 = 2343;
    for(int i = x17; i < x18; i++)
    {
        for(int j = y13; j < y14; j++) grid_map[i][j] = 0;//右垂直通路
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
