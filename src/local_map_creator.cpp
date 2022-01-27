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
    sub_laser_scan = nh.subscribe("corrected_scan",10,&Local_Map_Creator::laser_scan_callback,this);
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

    static bool first = true;
    if (!first) return;
    ROS_INFO_STREAM("recieved data size : " << msg->ranges.size());
    bool f = false;
    double dist = 0.30;
    int counter = 0;

    for(size_t i=0;i<4;i++){
       pole_min_idx[i] = 0;
       pole_max_idx[i] = 0;
    }

    for (size_t i = 0; i < msg->ranges.size(); i++) {
        if (scan_data.ranges[i] < dist && !f) {
            f = true;
            ROS_INFO_STREAM("from " << i);
            if(i==0)
               pole_min_idx[counter] = i;
            else
               pole_min_idx[counter] = i-10;
            if(i > 1000){
               pole_min_idx[counter] = i-10;
               pole_max_idx[counter] = 1081;
            }
        }
        if (scan_data.ranges[i] >= dist && f) {
            f = false;
            ROS_INFO_STREAM("     " << i << " end");
            pole_max_idx[counter] = i+10;
        }

        if((counter==0 || counter == 3) && pole_max_idx[counter] - pole_min_idx[counter] > 20)
            counter++;
        if((counter==1 || counter == 2) && pole_max_idx[counter] - pole_min_idx[counter] > 30)
            counter++;
    }

    ROS_INFO_STREAM("Auto detector");
    for(size_t i=0;i<4;i++){
        ROS_INFO_STREAM("from " << pole_min_idx[i]);
        ROS_INFO_STREAM("     " << pole_max_idx[i] << " end");
    }
    first = false;
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

int Local_Map_Creator::get_radius(int n)//柱のある角度のセンサーデータカット
{
    int right_back_min_idx = pole_min_idx[0];
    int right_back_max_idx = pole_max_idx[0];
    int right_forward_min_idx = pole_min_idx[1];
    int right_forward_max_idx = pole_max_idx[1];
    int left_back_min_idx = pole_min_idx[2];
    int left_back_max_idx = pole_max_idx[2];
    int left_forward_min_idx = pole_min_idx[3];
    int left_forward_max_idx = pole_max_idx[3];
    constexpr int alliance = 5;

    int radius = std::round(scan_data.ranges[n]/resolution);;
    int radius_limit = (column/2)-1;

    int proxy = -1;
    auto calc_proxy = [&](size_t min_idx, size_t max_idx, size_t current_idx) -> int {
        if (current_idx < min_idx || max_idx < current_idx) return proxy;
        assert(0 <= static_cast<int>(min_idx) - alliance || max_idx + alliance < scan_data.ranges.size());
        if (static_cast<int>(min_idx) - alliance < 0) return max_idx + alliance;
        if (max_idx + alliance >= scan_data.ranges.size()) return min_idx - alliance;
        size_t middle = (min_idx + max_idx) / 2;
        if (current_idx < middle) return min_idx - alliance;
        return max_idx + alliance;
    };
    proxy = calc_proxy(right_back_min_idx, right_back_max_idx, n);
    proxy = calc_proxy(right_forward_min_idx, right_forward_max_idx, n);
    proxy = calc_proxy(left_back_min_idx, left_back_max_idx, n);
    proxy = calc_proxy(left_forward_min_idx, left_forward_max_idx, n);
    proxy = calc_proxy(515, 532, n);
    if (proxy != -1 && (proxy < 0 || 1080 < proxy)) {
        ROS_ERROR_STREAM(proxy << " is out of range. " << n);
    }
    if (proxy != -1) radius = std::round(scan_data.ranges.at(proxy)/resolution);

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

