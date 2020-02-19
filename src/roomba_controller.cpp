#include "roomba_controller/roomba_controller.h"
//using namespace std;


RoombaController::RoombaController():private_nh("~")
{
    //parameter
    private_nh.parm("hz",hz,{10});

    //subscriber
    sub_odometry = n.subscribe("/roomba/odometry",1,odometry_callback);

    //publisher
    pub_control = n.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control",1,this);
}


void RoombaController::odometry_callback(const nav_msgs::Odometry::ConstPtr& odo)
{
  current_odometry = *odo;
}

void RoombaController::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        roomba_500driver_meiji::RoombaCtrl control;
        control.mode = 11;//roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
        if(current_odometry.pose.pose.position.z<0)
            current_odometry.pose.pose.position+=M_PI*2;
        do
        {
            init_odometry.pose.pose.position.x=current_odometry.pose.pose.position.x;
            init_odometry.pose.pose.position.y=current_odometry.pose.pose.position.y;
            init_odometry.pose.pose.position.z=current_odometry.pose.pose.position.z;
            init_odometry.pose.pose.position.orientation=current_odometry.pose.pose.position.orientation;
        } while(current_odometry.pose.pose.position.x==0.0)
        if(1/*fabs(init_odometry.pose.pose.position.orientation-current_odometry.pose.pose.position.orientation) <=*/ )
        {
           control.cntl.linear.x = 0.0;
           control.cntl.angular.z=0.1;
        }
        else
        {
           control.cntl.linear.x = 0.1;
           control.cntl.angular.z = 0.0;
        }
        pub_control.publish(control);
        ros::spinOnce();
        cout<< "current_odometry:" << current_odometry.pose.pose.position << endl;
        cout<<"init_odometry:"<<endl;
        cout << init_odometry.pose.pose.position << endl;
        loop_rate.sleep();
    }
}

int main (int argc, char **argv)
{
  ros::init(argc,argv,"roomba_controller");
  RoombaController roomba_controller;
  roomba_controller.process();
  return 0;
}
