#include <roomba_controller/roomba_controller.h>
//using namespace std;


RoombaController::RoombaController:private_nh("~")
{
    //parameter
    private_nh.parm("hz",hz,{10});
    private_nh.parm("sleeping_length",sleeping_length,{1.0});
    private_nh.parm("phase",phase,{0});
    private_nh.parm("theta",theta,{0});
    private_nh.parm("init_theta",init_theta,{0});


    //subscriber
    sub_odometry = n.subscribe("/roomba/odometry",1,odometry_callback);
    sub_laser = n.subscribe("/scan",1,laser_callback);

    //publisher
    pub_control = n.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control",1);

}


void RoombaController::odometry_callback(const nav_msgs::Odometry::ConstPtr& odo)
{
  current_odometry = *odo;
}

void RoombaController::laser_callback(const sensor_msgs::LaserScan::ConstPtr& lsr)
{
    laser = *lsr;
}

roomba_500driver_meiji::RoombaCtrl RoombaController::go_straight()
{
    roomba_500driver_meiji::RoombaCtrl control;
    if(sqrt(pow(init_odometry.pose.pose.position.x-odometry.pose.pose.position.x,2)+pow(init_odometry.pose.pose.y-odometry.pose.pose.y,2))<=3.0)
    {
        control.cntl.linear.x=0.2;
        control.cntl.angular.z=0.0;
    }
    else
    {
        init_theta = tf::getYaw(odometry.pose.pose.orientation);
        init_odometry.pose.pose.position.x=odometry.pose.pose.position.x;
        init_odometry.pose.pose.position.y=odometry.pose.pose.position.y;
        phase=2;
    }
    return control;
}

roomba_500driver_meiji::RoombaCtrl RoombaController::turn_a_round()
{
    roomba_500driver_meiji::RoombaCtrl control;
    if(!(fabs(init_theta-theta)<0.01*M_PI))
    {
        control.cntl.linear.x=0.0;
        control.cntl.angular.z=0.1;
    }
    else
    {
        phase=3;
    }
    return control;
}

roomba_500driver_meiji::RoombaCtrl RoombaController::laser_go()
{
    roomba_500driver_meiji::RoombaCtrl control;
    if(!laser.ranges.empty())
    {
        if(laser.ranges[540]>=0.6)
        {
            control.cntl.linear.x=0.1;
            control.cntl.angular.z=0;
        }
        else
        {
            control.cntl.linear.x=0.0;
            control.cntl.angular.z=0.0;
        }
    }
    return control;
}



void RoombaController::process()
{
    ros::Rate loop_rate(hz);
    ros::Rate sleep_rate(sleeping_length);
    while(ros::ok())
    {
        roomba_500driver_meiji::RoombaCtrl control;
        theta = tf::getYaw(odometry.pose.pose.orientation);
        if(theta<0)
            theta+= M_PI*2;
        switch(phase)
        {
            case(0):
                control.cntl.linear.x=0.0;
                control.cntl.angular.z=0.0;
                sleep_rate.sleep();
                phase=1;
                break;
            case(1):
                control=go_straight();
                break;
             case(2):
                control=turn_a_round();
             case(3):
                control=laser_go();
        }

        control.mode = roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
        pub_control.publish(control);
        ros::spinOnce();
        std::cout<< "current_odometry:" << current_odometry.pose.pose.position << std::endl;
        std::cout<<"init_odometry:"<<std::endl;
        std::cout << init_odometry.pose.pose.position << std::endl;
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
