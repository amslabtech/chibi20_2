#include "local_map_creator/local_map_creator.h"
Local_Map_Creator::Local_Map_Creator():private_nh("~")
{
    //parameter
    private_nh.param("hz",hz,{1});
    private_nh.param("width",width,{5});
    private_nh.param("height",height,{5});
    private_nh.param("resolution",resolution,{0.05});
    private_nh.param("radius_limit",radius_limit,{49});

    //subscriber
    sub_laser_scan = nh.subscribe("scan",10,&Local_Map_Creator::laser_scan_callback,this);
    sub_current_pose = nh.subscribe("estimated_pose",10,&Local_Map_Creator::current_pose_callback,this);
    //publisher
    pub_local_map = nh.advertise<nav_msgs::OccupancyGrid>("local_map",1);
}

void Local_Map_Creator::current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
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
    // grid_map.clear();
    grid_map.resize(row,std::vector<int>(column,-1));
    for(int i = 0; i < number_of_laser; i++) convert_coordinate(i);
    convert_grid_map();
    pub_local_map.publish(local_map);
}

// int Local_Map_Creator::get_radius(int n)//一定距離内のセンサーデータカット
// {
//     int radius;
//     int radius_limit = (column/2)-1;
//     double bar_limit = 0.00;
//     bool is_bar = true;
//     bool fliper = true;
//     int i= 0;
//     while(is_bar)
//     {
//         if(scan_data.ranges[n+i] > bar_limit)
//         {
//             radius = (int)(scan_data.ranges[n+i]/resolution);
//             is_bar = false;
//         }
//         if((n+i >= number_of_laser)||(n+i < 0)) fliper = !fliper;
//         if(fliper) i++;
//         else i--;
//     }
//     if(radius < radius_limit)
//     {
//         is_edge = false;
//         return radius;
//     }
//     else
//     {
//         is_edge = true;
//         return radius_limit;
//     }
// }

int Local_Map_Creator::get_radius(int n)//柱のある角度のセンサーデータカット
{
    int radius = 0;
    int radius_limit = (column/2)-1;
    radius = (int)(scan_data.ranges[n]/resolution);
    if(316 <= n)//右前の柱
    {
        if(n <= 323) radius = (int)(scan_data.ranges[314]/resolution);
        else if(n <= 332) radius = (int)(scan_data.ranges[334]/resolution);
    }

    if(764 <= n)//左前の柱
    {
        if(n <= 780) radius = (int)(scan_data.ranges[758]/resolution);
        else if(n <= 798) radius = (int)(scan_data.ranges[806]/resolution);
    }

    if(28 <= n)//右後の柱
    {
        if(n <= 48) radius = (int)(scan_data.ranges[26]/resolution);
        else if(n <= 68) radius = (int)(scan_data.ranges[70]/resolution);
    }

    if(1040 <= n)//左後の柱
    {
        if(n <= 1056) radius = (int)(scan_data.ranges[1038]/resolution);
        else if(n <= 1074) radius = (int)(scan_data.ranges[1078]/resolution);
    }

    // if(radius < 3) radius = radius_limit;

    if(radius < radius_limit)
    {
        is_edge = false;
        return radius;
    }
    else
    {
        is_edge = true;
        return radius_limit;
    }
}

void Local_Map_Creator::convert_coordinate(int i)
{
    double theta = (225-(i/4.0))*M_PI/180;
    for(int r = 0; r < get_radius(i); r++)
    {
        grid_map[(int)(r*cos(theta)+row/2)][(int)(r*sin(theta)+column/2)] = 0;
    }
    grid_map[(int)(get_radius(i)*cos(theta)+row/2)][(int)(get_radius(i)*sin(theta)+column/2)] = 100;

    if(is_edge)
    {
        grid_map[(int)(get_radius(i)*cos(theta)+row/2)][(int)(get_radius(i)*sin(theta)+column/2)] =0;
    }
}

void Local_Map_Creator::convert_grid_map()
{
    local_map.data.clear();
    local_map.header.frame_id = "map";
    local_map.header.stamp = ros::Time::now();
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

void Local_Map_Creator::add_frame_local_map()
{
    // tf2_ros::TransformBroadcaster tfb;
    // geometry_msgs::TransformStamped tf_local_map;
    // tf_local_map.header.frame_id = "map";
    // tf_local_map.child_frame_id = "local_map";
    // tf_local_map.transform.translation.x = current_pose.pose.position.x;
    // tf_local_map.transform.translation.y = current_pose.pose.position.y;
    // tf_local_map.transform.translation.z = 0;
    // tf_local_map.transform.rotation = current_pose.pose.orientation;
    // tf_local_map.header.stamp = ros::Time::now();
    // tfb.sendTransform(tf_local_map);
}

void Local_Map_Creator::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        // add_frame_local_map();
        ros::spin();
        loop_rate.sleep();
    }
}

int main (int argc, char **argv)
{
    ros::init(argc,argv,"local_map_creator");
    Local_Map_Creator local_map_creator;
    local_map_creator.process();
    return 0;
}

