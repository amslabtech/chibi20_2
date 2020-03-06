#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D"
#include "sensor_msgs/LaserScan.h"

class Particle
{
public:
    Particle(){
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        quaternionTFToMsg(tf::createQuaternionFromYaw(0),pose.pose.orientation);
    }

    void init(int width,int height,float resolution,geometry_msgs::Pose origin){
        pose.pose.position.x = (rand() % width)  * resolution + origin.position.x;
        pose.pose.posotion.y = (rand() % height) * resolution + origin.position.y;
        quaternionTFToMsg(tf::createQuaternionFromYaw((rand() % 360) / 180     * M_PI - M_PI),pose.pose.orientation);
    }

    geometry_msgs::PoseStamped pose;
};

nav_msgs::OcupancyGrid map;
std::vector<Particle> particle
const int N   //Particle

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    map = *map;

    for(int i = 0; i < N; i++){
        Particle p;
        do{
            p.init(map.info.width,map.info.height,map.info.resolution,map.info.origin);
        }
        while(p.pose.pose.position.x != 0 | p.pose.pose.position.y != 0){
        }
    }
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& lsr)
{
    lsr = *lsr;
}
