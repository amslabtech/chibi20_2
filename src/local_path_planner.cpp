#include "local_path_planner/local_path_planner.h"

Dynamic_Window_Approch::Dynamic_Window_Approch():private_nh("~")
{
    //parameter
    private_nh.param("max_linear_velocity",max_linear_velocity,{0.5});
    private_nh.param("max_angular_velocity",max_angular_velocity,{4.25});
    private_nh.param("max_linear_acceleration",max_linear_acceleration,{0.1});
    private_nh.param("max_angular_acceleration",max_angular_acceleration,{0.85});
    private_nh.param("hz",hz,{1});
    private_nh.param("dx",dx,{0.05});
    private_nh.param("da",da,{0.425});
    private_nh.param("dt",dt,{0.1});
    private_nh.param("sim_time",sim_time,{2});
    private_nh.param("sigma",sigma,{1.0/3.0});
    private_nh.param("k_heading",k_heading,{1.0});
    private_nh.param("k_distance",k_distance,{1.0});
    private_nh.param("k_velocity",k_velocity,{1.0});

    //subscriber
    sub_local_map = nh.subscribe("local_map",10,&Dynamic_Window_Approch::local_map_callback,this);
    sub_local_goal = nh.subscribe("local_goal",10,&Dynamic_Window_Approch::local_goal_callback,this);
    sub_way_blocked = nh.subscribe("way_blocked_checker",10,&Dynamic_Window_Approch::way_blocked_callback,this);
    sub_estimated_pose = nh.subscribe("estimated_pose",10,&Dynamic_Window_Approch::estimated_pose_callback,this);
    sub_roomba_odometry = nh.subscribe("/roomba/odometry",10,&Dynamic_Window_Approch::roomba_odometry_callback,this);
    //publisher
    pub_roomba_ctrl = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba_ctrl",1);
    pub_virtual_path = nh.advertise<nav_msgs::Path>("virtual_path",1);
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
    mini_point.linear = 0;//max_linear_velocity*-1; 動的障害物を検知できたらバックも
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
    list[n].virtual_path.poses.push_back(simulate(0.0,0.0,0.0));
    for(double time = 0;time < sim_time; time += dt)
    {
       list[n].virtual_path.poses.push_back(simulate(time,v,omega));
    }
    list[n].virtual_path.header.frame_id = "map";
    pub_virtual_path.publish(list[n].virtual_path);
}

geometry_msgs::PoseStamped Dynamic_Window_Approch::simulate(double time,double v,double omega)
{
    geometry_msgs::PoseStamped sim_pose;
    double theta = omega*time;
    if(fabs(omega) < 0.01)
    {
        if(omega > 0) omega = 0.01;
        else omega = -0.01;
    }
    sim_pose.pose.position.x = v*(time*cos(theta) - sin(theta)/omega)/omega;
    sim_pose.pose.position.y = v*(time*sin(theta) + cos(theta)/omega)/omega;
    quaternionTFToMsg(tf::createQuaternionFromRPY(0.0,0.0,theta),sim_pose.pose.orientation);
    // std::cout<<"sim_pose: "<<sim_pose<<std::endl;
    return sim_pose;
}

void Dynamic_Window_Approch::evaluation(const geometry_msgs::PoseStamped& virtual_pose,int counter,unsigned int step)
{
    //g(v,omega) = sigma(k_h*heading(v,omega)+beta*dist(v,omega)+gamma*velocity(v,omega))
   double x = estimated_pose.pose.position.x + virtual_pose.pose.position.x;
   double y = estimated_pose.pose.position.y + virtual_pose.pose.position.y;
   list[counter].heading_score +=  heading(virtual_pose,x,y);
   // std::cout<<"step_head: "<<step<<","<<list[counter].heading_score<<std::endl;
   list[counter].distance_score +=  distance(virtual_pose,x,y);
   // list[counter].velocity_score +=  velocity(virtual_pose,counter,step);
}

double Dynamic_Window_Approch::heading(const geometry_msgs::PoseStamped& pose,int x,int y)
{
    double goal_theta = std::atan2(local_goal.pose.position.y-y,local_goal.pose.position.x-x);
    double theta = tf::getYaw(estimated_pose.pose.orientation);
    // std::cout<<"goal_theta,theta"<<goal_theta*180/M_PI<<","<<theta*180/M_PI<<std::endl;
    // std::cout<<"score: "<<(1.0-(fabs(theta-goal_theta)/M_PI))<<std::endl;
    return (1.0-(fabs(theta-goal_theta)/M_PI));
}

double Dynamic_Window_Approch::distance(const geometry_msgs::PoseStamped& posei,int x,int y)
{
    double virtual_distance = sqrt(pow(x-local_goal.pose.position.x,2)+pow(y-local_goal.pose.position.y,2));
    std::cout<<"virtual_distance,current_distance: "<<virtual_distance<<","<<current_distance/current_distance<<std::endl;
    std::cout<<"score: "<<virtual_distance/current_distance<<std::endl;
    return virtual_distance/current_distance;
}

// double Dynamic_Window_Approch::velocity(const geometry_msgs::PoseStamped& pose,int counter,unsigned int step)
// {
//     if(step == 0)
//         return sqrt(pow(pose.pose.position.x,2)+pow(pose.pose.position.y,2));
//     else
//         return sqrt(pow(pose.pose.position.x-list[counter].virtual_path.poses[step-1].pose.position.x,2)+pow(pose.pose.position.y-list[counter].virtual_path.poses[step-1].pose.position.y,2));
// }
//

void Dynamic_Window_Approch::normalize_score(int n)
{

    list[n].heading_score /= list[n].virtual_path.poses.size();
    // std::cout<<"final_heading: "<<list[n].heading_score;
}

void Dynamic_Window_Approch::consider_local_path()
{
    int linear_step = (int)(1+((window[1].linear-window[0].linear)/dx));
    int angular_step = (int)(1+((window[1].angular-window[0].angular)/da));
    list.reserve(linear_step*angular_step);
    int counter = 0;
    // std::cout<<"linear_step"<<linear_step<<std::endl;
    // std::cout<<"angular_step"<<angular_step<<std::endl;
    ROS_INFO("3");
    for(double linear = window[0].linear; linear < window[1].linear; linear += dx)
    {
        for(double angular = window[0].angular; angular < window[1].angular; angular += da)
        {
            Considering_list add = { };
            list.push_back(add);
            create_virtual_path(linear,angular,counter);
            current_distance = sqrt(pow(local_goal.pose.position.x-estimated_pose.pose.position.x,2)+pow(local_goal.pose.position.y-estimated_pose.pose.position.y,2));
            for(unsigned int step = 0; step < list[counter].virtual_path.poses.size(); step ++)
            {
                evaluation(list[counter].virtual_path.poses[step],counter,step);
            }
            normalize_score(counter);
            // list[counter].total_score = (k_heading*list[counter].heading_score)+(k_distance*list[counter].distance_score)+(k_velocity*list[counter].velocity_score);
            std::cout<<"counter: "<<counter<<std::endl;
            std::cout<<"heading_score: "<<list[counter].heading_score<<std::endl;
            std::cout<<"distance_score: "<<list[counter].distance_score<<std::endl;
            std::cout<<"velocity_score: "<<list[counter].velocity_score<<std::endl;
            counter++;
        }
    }
}

// void Dynamic_Window_Approch::decide_local_path()
// {
//     int best_path_number = -1;
//     double best_score = -3;
//     for(unsigned int i = 0; i < list.size(); i++)
//     {
//         if(list[i].total_score > best_score)
//         {
//             best_score = list[i].total_score;
//             best_path_number = i;
//         }
//     }
//     Component best_velocity = list[best_path_number].velocity;
//     roomba_500driver_meiji::RoombaCtrl movement;
//     movement.mode = roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
//     movement.cntl.linear.x = sigma*best_velocity.linear;
//     movement.cntl.angular.z = sigma*best_velocity.angular;
//     pub_roomba_ctrl.publish(movement);
// }
//
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
            // decide_local_path();
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
