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

        //parameter
        double hz;




        //member
        ros::NodeHndle n;
        ros::Publisher pub_controll;
        ros::Subscriber sub_odometry;
        nav_msgs::Odometry current_odometry;
};

#endif
