#include "local_path_planner/local_path_planner.h"

Dynamic_Window_Approch::Dynamic_Window_Approch():private_nh("~")
{
    //parameter
    private_nh.param("max_linear_velocity",max_linear_velocity,{0.5});
    private_nh.param("max_angular_velocity",max_angular_velocity,{30*M_PI/180});//4.25
    private_nh.param("max_linear_acceleration",max_linear_acceleration,{0.05});
    private_nh.param("max_angular_acceleration",max_angular_acceleration,{30*M_PI/180});
    private_nh.param("hz",hz,{10});
    private_nh.param("dx",dx,{0.05});
    private_nh.param("da",da,{5.0*M_PI/180});
    private_nh.param("dt",dt,{0.25});
    private_nh.param("sim_time",sim_time,{5.0});
    private_nh.param("sigma_velocity",sigma_linear,{1.0});
    private_nh.param("sigma_angular",sigma_angular,{2.0});
    private_nh.param("k_heading",k_heading,{1.0});
    private_nh.param("k_distance",k_distance,{1.0});
    private_nh.param("k_velocity",k_velocity,{1.0});
    private_nh.param("pick_up_time",pick_up_time,{4.0});

    //subscriber
    sub_local_map = nh.subscribe("local_map",10,&Dynamic_Window_Approch::local_map_callback,this);
    sub_local_goal = nh.subscribe("local_goal",10,&Dynamic_Window_Approch::local_goal_callback,this);
    sub_way_blocked = nh.subscribe("way_blocked_checker",10,&Dynamic_Window_Approch::way_blocked_callback,this);
    sub_estimated_pose = nh.subscribe("estimated_pose",10,&Dynamic_Window_Approch::estimated_pose_callback,this);
    sub_roomba_odometry = nh.subscribe("/roomba/odometry",10,&Dynamic_Window_Approch::roomba_odometry_callback,this);
    //publisher
    pub_roomba_ctrl = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control",1);
    pub_virtual_path = nh.advertise<nav_msgs::Path>("virtual_path",1);
    pub_best_path = nh.advertise<nav_msgs::Path>("best_path",1);
    pub_eliminated_path = nh.advertise<nav_msgs::Path>("eliminated_path",1);
    pub_obstacled_grid = nh.advertise<geometry_msgs::PointStamped>("obstacled_grid",1);
}

void Dynamic_Window_Approch::roomba_odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_pose.pose = msg->pose.pose;
    if(!is_odometry_recieved) past_pose = current_pose;
    current_velocity.linear = sqrt(pow(current_pose.pose.position.x - past_pose.pose.position.x,2) + pow(current_pose.pose.position.y - past_pose.pose.position.y,2));
    current_velocity.angular = tf::getYaw(current_pose.pose.orientation) - tf::getYaw(past_pose.pose.orientation);
    past_pose = current_pose;
    is_odometry_recieved = true;
}

void Dynamic_Window_Approch::local_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
     local_map = *msg;
     is_map_recieved = true;
}

void Dynamic_Window_Approch::estimated_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    estimated_pose = *msg;
}

void Dynamic_Window_Approch::way_blocked_callback(const std_msgs::Bool::ConstPtr& msg)
{

}

void Dynamic_Window_Approch::local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_goal = *msg;
    is_goal_recieved = true;
}
void Dynamic_Window_Approch::create_dynamic_window()
{
    window.clear();
    spec_window();
    reachable_window();
    obstacle_window();
    show_window();
}

void Dynamic_Window_Approch::show_window()
{
    std::cout<<"linear: "<<window[0].linear<<"~"<<window[1].linear<<std::endl;
    std::cout<<"angular: "<<window[0].angular<<"~"<<window[1].angular<<std::endl;
}

void Dynamic_Window_Approch::spec_window()
{
    Component mini_point;
    mini_point.linear = 0;
    mini_point.angular = max_angular_velocity*-1;
    window.push_back(mini_point);
    Component max_point;
    max_point.linear = max_linear_velocity;
    max_point.angular = max_angular_velocity;
    window.push_back(max_point);
}

void Dynamic_Window_Approch::reachable_window()
{
    Component mini_point;
    mini_point.linear = current_velocity.linear - max_linear_acceleration*sim_time;
    mini_point.angular =  current_velocity.angular - max_angular_acceleration*sim_time;
    if(window[0].linear < mini_point.linear) window[0].linear = mini_point.linear;
    if(window[0].angular < mini_point.angular) window[0].angular = mini_point.angular;
    Component max_point;
    max_point.linear = current_velocity.linear + max_linear_acceleration*sim_time;
    max_point.angular =  current_velocity.angular + max_angular_acceleration*sim_time;
    if(window[1].linear > max_point.linear) window[1].linear = max_point.linear;
    if(window[1].angular > max_point.angular) window[1].angular = max_point.angular;
}

void Dynamic_Window_Approch::obstacle_window()
{
}

void Dynamic_Window_Approch::create_virtual_path(double v,double omega,int n)
{
    list[n].virtual_path.poses.clear();
    list[n].velocity.linear = v;
    list[n].velocity.angular = omega;
    geometry_msgs::PoseStamped init_pose ={};
    quaternionTFToMsg(tf::createQuaternionFromRPY(0.0,0.0,0.0),init_pose.pose.orientation);
    list[n].virtual_path.poses.push_back(init_pose);
    for(double time = 0;time < sim_time; time += dt)
    {
       list[n].virtual_path.poses.push_back(simulate(time,v,omega,n));
    }
    list[n].virtual_path.header.frame_id = "map";
    pub_virtual_path.publish(list[n].virtual_path);
}

geometry_msgs::PoseStamped Dynamic_Window_Approch::simulate(double time,double v,double omega,int n)
{
    geometry_msgs::PoseStamped sim_pose;
    double theta = omega*time;
    sim_pose.pose = list[n].virtual_path.poses.back().pose;
    sim_pose.pose.position.x += v*dt*cos(theta);
    sim_pose.pose.position.y += v*dt*sin(theta);
    quaternionTFToMsg(tf::createQuaternionFromRPY(0.0,0.0,theta),sim_pose.pose.orientation);
    // std::cout<<"sim_pose: "<<sim_pose<<std::endl;
    return sim_pose;
}

void Dynamic_Window_Approch::evaluation(int counter)
{
    //g(v,omega) = sigma(k_h*heading(v,omega)+beta*dist(v,omega)+gamma*velocity(v,omega))
    int pick_up_step = (int)(pick_up_time/dt);
    geometry_msgs::PoseStamped pick_up_pose = list[counter].virtual_path.poses[pick_up_step];
    double estimated_theta = tf::getYaw(estimated_pose.pose.orientation);
    double x = estimated_pose.pose.position.x + pick_up_pose.pose.position.x*cos(estimated_theta) - pick_up_pose.pose.position.y*sin(estimated_theta);
    double y = estimated_pose.pose.position.y + pick_up_pose.pose.position.x*sin(estimated_theta) + pick_up_pose.pose.position.y*cos(estimated_theta);
    list[counter].heading_score = heading(x,y,counter);
    list[counter].distance_score = distance(x,y);
    list[counter].velocity_score = list[counter].velocity.linear/max_linear_velocity;
    list[counter].total_score = (k_heading*list[counter].heading_score)+(k_distance*list[counter].distance_score)+(k_velocity*list[counter].velocity_score);
}

double Dynamic_Window_Approch::heading(double x,double y,int counter)
{
    double goal_theta = std::atan2(local_goal.pose.position.y-y,local_goal.pose.position.x-x);
    double theta = tf::getYaw(estimated_pose.pose.orientation)+list[counter].velocity.angular;
    double heading_theta = fabs(theta -goal_theta);
    if(heading_theta > M_PI) heading_theta = 2*M_PI - heading_theta;
    if(heading_theta < -1*M_PI) heading_theta = 2*M_PI + heading_theta;
    return (1.0-(fabs(heading_theta)/M_PI));
}

double Dynamic_Window_Approch::distance(double x,double y)
{
    double virtual_distance = sqrt(pow(x-local_goal.pose.position.x,2)+pow(y-local_goal.pose.position.y,2));
    // double current_distance = sqrt(pow(estimated_pose.pose.position.x-local_goal.pose.position.x,2)+pow(estimated_pose.pose.position.y-local_goal.pose.position.y,2));
    double score =  (1-(virtual_distance/10.0));
    if(score >= 0) return score;
    else return 0;
}

void Dynamic_Window_Approch::consider_local_path()
{
    // int linear_step = (int)(1+((window[1].linear-window[0].linear)/dx));
    // int angular_step = (int)(1+((window[1].angular-window[0].angular)/da));
    // list.reserve(linear_step*angular_step);
    int counter = 0;
    // std::cout<<"linear_step"<<linear_step<<std::endl;
    // std::cout<<"angular_step"<<angular_step<<std::endl;
    // ROS_INFO("3");
    for(double linear = window[0].linear; linear < window[1].linear; linear += dx)
    {
        for(double angular = window[0].angular; angular < window[1].angular; angular += da)
        {
            Considering_list add = { };
            list.push_back(add);
            create_virtual_path(linear,angular,counter);
            evaluation(counter);
            // normalize_score(counter);
            // std::cout<<"counter: "<<counter<<std::endl;
            // std::cout<<"heading_score: "<<list[counter].heading_score<<std::endl;
            // std::cout<<"distance_score: "<<list[counter].distance_score<<std::endl;
            // std::cout<<"velocity_score: "<<list[counter].velocity_score<<std::endl;
            // std::cout<<"total_score: "<<list[counter].total_score<<std::endl;
            counter++;
        }
    }
}

void Dynamic_Window_Approch::eliminate_obstacle_path()
{
    int row = (int)(local_map.info.height/local_map.info.resolution);
    int column = (int)(local_map.info.width/local_map.info.resolution);
    grid_map.resize(row,std::vector<int>(column));
    for(int i = 0; i < row; i++)
    {
        for(int j = 0; j < column; j++)
        {
            grid_map[i][j] = local_map.data[j*row+i];
        }
    }
    for(int i = 0; i < (int)list.size(); i++)
    {
        list[i].is_obstacled = false;
        for(int j = 0; j < (int)list[i].virtual_path.poses.size(); j++)
        {
            // std::cout<<"i,j: "<<i<<" , "<<j<<std::endl;
            int x = (int)((list[i].virtual_path.poses[j].pose.position.x/local_map.info.resolution)+row/2);
            int y = (int)((list[i].virtual_path.poses[j].pose.position.y/local_map.info.resolution)+column/2);
            if((j < (pick_up_time+1)/dt) && (!list[i].is_obstacled)) list[i].is_obstacled = check_obstacle(x,y);
        }

        if(list[i].is_obstacled)
        {
            pub_eliminated_path.publish(list[i].virtual_path);
            // std::cout<<"number: "<<i<<std::endl;
        }
    }
}

bool Dynamic_Window_Approch::check_obstacle(int x,int y)
{
    int obstacle_range = 4;
    // std::cout<<"x,y,range: "<<x<<" , "<<y<<" , "<<obstacle_range<<std::endl;
    for(int i = x-obstacle_range; i <=  x+obstacle_range; i++)
    {
        for(int j = y-obstacle_range; j <= y+obstacle_range; j++)
        {
            if(grid_map[i][j] == 100) return true;
            // geometry_msgs::PointStamped obstacled;
            // obstacled.point.x = i*local_map.info.resolution-50;
            // obstacled.point.y = j*local_map.info.resolution-50;
            // obstacled.header.frame_id = "map";
            // pub_obstacled_grid.publish(obstacled);
        }
    }
    return false;
}

void Dynamic_Window_Approch::decide_local_path()
{
    int best_path_number = -1;
    double best_score = -1.0;
    for(int i = 0; i < list.size(); i++)
    {
        if((list[i].total_score > best_score) && (!list[i].is_obstacled))
        {
            best_score = list[i].total_score;
            best_path_number = i;
        }
    }
    Component best_velocity = list[best_path_number].velocity;
    roomba_500driver_meiji::RoombaCtrl movement;
    movement.mode = roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
    if(best_path_number >= 0)
    {
        pub_best_path.publish(list[best_path_number].virtual_path);
        std::cout<<"best_score: "<<best_score<<std::endl;
        std::cout<<"number: "<<best_path_number<<" linear: "<<list[best_path_number].velocity.linear<<" angular: "<<list[best_path_number].velocity.angular*180/M_PI<<std::endl;
        movement.cntl.linear.x = sigma_linear*best_velocity.linear;
        movement.cntl.angular.z = sigma_angular*best_velocity.angular;
        std::cout<<"vl,va: "<<movement.cntl.linear.x<<" , "<<movement.cntl.angular.z<<std::endl;
    }
    else
    {
        movement.cntl.linear.x = 0.0;
        movement.cntl.angular.z = 0.2;
        std::cout<<"no_path"<<std::endl;
    }
    pub_roomba_ctrl.publish(movement);
}

void Dynamic_Window_Approch::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        if(is_map_recieved&&is_goal_recieved&&is_odometry_recieved)
        {
            list.clear();
            create_dynamic_window();
            consider_local_path();
            eliminate_obstacle_path();
            decide_local_path();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"local_path_planner");
    Dynamic_Window_Approch dynamic_window_approch;
    dynamic_window_approch.process();
    return 0;
}
