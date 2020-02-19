#include <ros/ros.h>
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
// using namespace std;

nav_msgs::Odometry odometry;
nav_msgs::Odometry init_odometry;
int phase=0;
double theta=0.0;
double init_theta=0.0;
int sleep_flag=0;

void odometry_callback(const nav_msgs::Odometry::ConstPtr& odo)
{
  odometry = *odo;
}

roomba_500driver_meiji::RoombaCtrl  sleep_init()
{
    roomba_500driver_meiji::RoombaCtrl control;
    sleep_flag++;
    if(sleep_flag>=60)
    {

        if (phase==0)
        {
            phase=1;
            init_theta=  tf::getYaw(odometry.pose.pose.orientation);
            theta=  tf::getYaw(odometry.pose.pose.orientation);
            init_odometry.pose.pose.position.x=odometry.pose.pose.position.x;
            init_odometry.pose.pose.position.y=odometry.pose.pose.position.y;
            control.cntl.linear.x=0;
            control.cntl.angular.z=0;
        }
        if(phase==2)
        {
            control.cntl.linear.x=0;
            control.cntl.angular.z=0.1;
            if(sleep_flag>=360)
                phase=3;
        }
    }
    return control;
}


roomba_500driver_meiji::RoombaCtrl  go_straight()
{
    roomba_500driver_meiji::RoombaCtrl control;
    // if(fabs(pow(init_odometry.pose.pose.position.x-odometry.pose.pose.position.x,2)+pow(init_odometry.pose.pose.position.y-odometry.pose.pose.position.y,2))<=9.0)
    if(fabs(init_odometry.pose.pose.position.x-odometry.pose.pose.position.x)<1.0)
    {
        control.cntl.linear.x=0.2;
        control.cntl.angular.z=0.0;
    }
    else
    {
        phase=2;
    }
    return control;
}

roomba_500driver_meiji::RoombaCtrl turn_a_round()
{
    roomba_500driver_meiji::RoombaCtrl control;
    if(init_theta<M_PI*-1*0.99)
    {
        init_theta=M_PI*-1*0.97;
    }
    if(!fabs(theta-init_theta<M_PI*0.01))
    {
        control.cntl.linear.x=0.0;
        control.cntl.angular.z=0.1;
    }
    else
    {
        phase=4;
    }

    return control;
}

int   main (int argc, char **argv)
{
  ros::init(argc,argv,"test");
  ros::NodeHandle n;
  ros::Publisher pub_control = n.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control",100);
  ros::Subscriber sub_odometry = n.subscribe("/roomba/odometry",100,odometry_callback);
  ros::Rate loop_rate(60);
  // do
  // {
  //     init_odometry.pose.pose.position.x=odometry.pose.pose.position.x;
  //     init_odometry.pose.pose.position.y=odometry.pose.pose.position.y;
      // init_theta=  tf::getYaw(odometry.pose.pose.orientation);
      // theta=  tf::getYaw(odometry.pose.pose.orientation);
  // }while(odometry.pose.pose.position.x==0);

  while(ros::ok())
  {
    roomba_500driver_meiji::RoombaCtrl control;
    control.mode = 11;//roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
    theta=  tf::getYaw(odometry.pose.pose.orientation);
    switch(phase)
    {
        case(0):
            control=sleep_init();
            break;
        case(1):
            control=go_straight();
            break;
        case(2):
            control=sleep_init();
            break;
        case(3):
            control=turn_a_round();
            break;
        default:
            break;
    }
    if(phase==4)
    {
         control.cntl.linear.x=0.0;
         control.cntl.angular.z=0.0;
    }
    control.mode = 11;//roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
    pub_control.publish(control);
    ros::spinOnce();
    loop_rate.sleep();
    std::cout << "theta:"<<theta << std::endl;
    std::cout << "init_theta:"<<init_theta << std::endl;
    std::cout << "phase:"<<phase << std::endl;
    std::cout << "odometry:"<<odometry.pose.pose.position << std::endl;
    // std::cout << "init_odometry:"<<init_odometry.pose.pose.position <<std:: endl;
  }

  return 0;
}
