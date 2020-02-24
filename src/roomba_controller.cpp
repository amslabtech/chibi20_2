// #include "roomba_controller/roomba_controller.h"
//using namespace std;
#include "roomba_controller/roomba_controller.h"

 RoombaController::RoombaController() : private_n("~")
{
    //parameter
    private_n.param("hz",hz,{10});
    private_n.param("sleeping_length",sleeping_length,{1.0});
    private_n.param("phase",phase,{0});
    private_n.param("theta",theta,{0});
    private_n.param("init_theta",init_theta,{0});
    private_n.param("turn_phase",turn_phase,{0});
    private_n.param("average_length",average_length,{0});
    private_n.param("center_number",center_number,{540});
    private_n.param("alpha",alpha,{5});



    //subscriber
    sub_odometry = n.subscribe("/roomba/odometry",1,&RoombaController::odometry_callback,this);
    sub_laser = n.subscribe("/scan",1,&RoombaController::laser_callback,this);

    //publisher
    pub_control = n.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control",1);

}


void RoombaController::odometry_callback(const nav_msgs::Odometry::ConstPtr& odo)
{
  odometry = *odo;
}

void RoombaController::laser_callback(const sensor_msgs::LaserScan::ConstPtr& lsr)
{
    laser = *lsr;
}

roomba_500driver_meiji::RoombaCtrl RoombaController::go_straight()
{
    roomba_500driver_meiji::RoombaCtrl control;
    if(sqrt(pow(init_odometry.pose.pose.position.x-odometry.pose.pose.position.x,2)+pow(init_odometry.pose.pose.position.y-odometry.pose.pose.position.y,2))<=3.0)
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
    if(theta-init_theta<=0)
        turn_phase=1;
    if((turn_phase==1)&&(theta-init_theta>0))
        phase=3;
    control.cntl.linear.x=0.0;
    control.cntl.angular.z=0.1;
    return control;
}

roomba_500driver_meiji::RoombaCtrl RoombaController::laser_go()
{
    roomba_500driver_meiji::RoombaCtrl control;
    if(!laser.ranges.empty())
    {
        average_length=0.0;
        for(int i=center_number-alpha;i<center_number+alpha;i++)
        {
            average_length+=laser.ranges[i];
        }
        average_length/=2*alpha;
        if(average_length>=0.5)
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
        std::cout<< "odometry:" << odometry.pose.pose.position << std::endl;
        std::cout<<"init_odometry:"<<std::endl;
        std::cout << init_odometry.pose.pose.position << std::endl;
        std::cout << "average_length:"<<average_length<<std::endl;
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
