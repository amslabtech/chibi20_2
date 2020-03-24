#include "local_map_creator/local_map_creator.h"
Local_Map_Creator::Local_Map_Creator():private_nh("~")
{
    //parameter
    private_nh.param("hz",hz,{1});
    private_nh.param("width",width,{5});
    private_nh.param("height",height,{5});
    private_nh.param("resolution",resolution,{0.05});
    private_nh.param("radius_limit",radius_limit,{99});

    //subscriber
    sub_laser_scan = nh.subscribe("scan",10,&Local_Map_Creator::laser_scan_callback,this);
    //publisher
    pub_local_map = nh.advertise<nav_msgs::OccupancyGrid>("local_map",1);
}

void Local_Map_Creator::laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // std::cout<<"recieved scan_data"<<std::endl;
    scan_data = *msg;
    create_local_map();
}

void Local_Map_Creator::create_local_map()
{
    row = (int)(height/resolution);
    column = (int)(width/resolution);
    grid_map.resize(row,std::vector<int>(column,-1));
    for(int i = 0; i < number_of_laser; i++) convert_coordinate(i);
    convert_grid_map();
    pub_local_map.publish(local_map);
}

int Local_Map_Creator::get_radius(int n)
{
    int radius;
    int radius_limit = (column/2)-1;
    // double average_length = (scan_data.ranges[n]+scan_data.ranges[n+1]+scan_data.ranges[n+2]+scan_data.ranges[n+3])/4;
    radius = (int)(scan_data.ranges[n]/resolution);
    // std::cout<<"radius: "<<radius<<std::endl;
    if(radius < radius_limit) return radius;
    else return radius_limit;
}

void Local_Map_Creator::convert_coordinate(int i)
{
    double theta = (i/4-135)*M_PI/180;
    // std::cout<<"r,theta: "<<get_radius(i)<<","<<(i-45)<<std::endl;
    for(int r = 0; r < get_radius(i); r++)
    {
        grid_map[(int)(r*cos(theta)+row/2)][(int)(r*sin(theta)+column/2)] = 0;
        // std::cout<<"i,j: "<<cos(theta)<<","<<sin(theta)<<std::endl;
    }
    if(get_radius(i) >= radius_limit-1)
    {
        grid_map[(int)(get_radius(i)*cos(theta)+row/2)][(int)(get_radius(i)*sin(theta)+column/2)] = 0;
    }
    else
    {
        grid_map[(int)(get_radius(i)*cos(theta)+row/2)][(int)(get_radius(i)*sin(theta)+column/2)] = 100;
    }
}

void Local_Map_Creator::convert_grid_map()
{
    local_map.data.clear();
    local_map.header.frame_id = "local_map";
    local_map.info.resolution = resolution;
    local_map.info.width = column;
    local_map.info.height = row;
    local_map.info.origin.position.x = -1*(double)height/2;
    local_map.info.origin.position.y = -1*(double)width/2;
    for(int i = 0; i < column; i++)
    {
        for(int j = 0; j < row; j++)
        {
            local_map.data.push_back(grid_map[i][j]);
        }
    }
}

void Local_Map_Creator::process()
{
    // ros::Rate loop_rate(hz);
    // while(ros::ok())
    // {
        ros::spin();
    //     loop_rate.sleep();
    // }
}

int main (int argc, char **argv)
{
    ros::init(argc,argv,"local_map_creator");
    Local_Map_Creator local_map_creator;
    local_map_creator.process();
    return 0;
}

