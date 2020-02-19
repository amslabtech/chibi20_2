#include "ros/ros.h"
#include <roomba_500driver_meiji/RoombaCtrl.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
// using namespace std;

nav_msgs::Odometry odometry;
nav_msgs::Odometry init_odometry;
int turn_flag=0;

void odometry_callback(const nav_msgs::Odometry::ConstPtr& odo)
{
  odometry = *odo;
}

int main (int argc, char **argv)
{
  ros::init(argc,argv,"test");
  ros::NodeHandle n;
  ros::Publisher pub_control = n.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control",100);
  ros::Subscriber sub_odometry = n.subscribe("/roomba/odometry",100,odometry_callback);
  ros::Rate loop_rate(10);
  do
  {
      init_odometry.pose.pose.position.x=odometry.pose.pose.position.x;
      init_odometry.pose.pose.position.y=odometry.pose.pose.position.y;
      init_odometry.pose.pose.orientation=odometry.pose.pose.orientation;
  }while(odometry.pose.pose.position.x==0);

  while(ros::ok())
  {
    roomba_500driver_meiji::RoombaCtrl control;
    if(turn_flag==0&&(fabs(pow(init_odometry.pose.pose.position.x-odometry.pose.pose.position.x,2)+pow(init_odometry.pose.pose.position.y-odometry.pose.pose.position.y,2))<=1.0))
   {
       control.mode = roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
       control.cntl.linear.x = 0.1;
       control.cntl.angular.z=0.0;
   }
   else
   {
     control.cntl.linear.x = 0.0;
     control.cntl.angular.z = 0.1;
     if(( odometry.pose.pose.orientation - init_odometry.pose.pose.orientation )>0)
         turn_flag=1;
   }
    control.cntl.linear.x=0.0;
    control.cntl.angular.z=0.0;
    pub_control.publish(control);
    ros::spinOnce();
    loop_rate.sleep();
    std::cout << "odometry:"<<odometry.pose.pose.position << std::endl;
    std::cout << "init_odometry:"<<init_odometry.pose.pose.position <<std:: endl;
  }

  return 0;
}
