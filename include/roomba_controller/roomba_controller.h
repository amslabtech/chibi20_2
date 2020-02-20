#ifdef _ROOMBA_CONTROLLER_H
#define _ROOMBA_CONTROLLER_H

#include "ros/ros.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include "tf/tf.h"
#include "sensor_msgs/LaserScan.h"

class RoombaContoroller
{
public:
        RoombaContoroller();
        void process();


private:
        //mrthod
        void odometry_callback(const nav_msgs::Odometry::ConstPtr&);
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr&);
        roomba_500driver_meiji::RoombaCtrl go_straight();
        roomba_500driver_meiji::RoombaCtrl turn_a_round();
        roomba_500driver_meiji::RoombaCtrl laser_go();

        //parameter
        double hz;
        double sleeping_length;
        int phase ;
        double theta;
        double init_theta;



        //member
        ros::NodeHndle n;
        ros::Publisher pub_controll;
        ros::Subscriber sub_odometry;
        ros::Subscriber sub_laser;
        nav_msgs::Odometry current_odometry;
        nav_msgs::Odometry init_odometry;
        sensor_msgs::LaserScan laser;

};

#endif
