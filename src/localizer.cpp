#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include <math.h>

class Particle
{
public:
    Particle();
    void p_init(int width,int height,float resolution,geometry_msgs::Pose origin);
    void p_motion_updata();
    void p_measurement_updata();
    void p_move();

    geometry_msgs::PoseStamped pose;

private;
    double weight;
};

nav_msgs::Odometry odometry;
nav_msgs::Odometry init_odometry;
nav_msgs::OcupancyGrid map;
sensor_msgs::LaserScan laser
geometry_msgs::Pose2D pose2d;
geometry_msgs::PoseArray poses;

std::vector<Particle> particle

const int N;    //Particle
float theta;    //Yaw
bool get_map = false;

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    map = *map;

    for(int i = 0; i < N; i++){
        Particle p;
        do{
            p.init(map.info.width,map.info.height,map.info.resolution,map.info.origin);
        }
        while(grid_data[p.pose.pose.position.x,p.pose.pose.position.y] != 0){
            particle.push_back(p);
        }
        poses.poses.push_back(p.pose.pose);
    }
    poses.header.frame_id = "map";

    get_map = true;
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& lsr)
{
    laser = *lsr;
}

void odometry_callback(const nav_msgs::Odomenry::ConstPtr& odo)
{
    odometry = *odo;
}

//乱数生成
float random()
{
    static int flag;

    if(flag == 0){
        srand((unsigned int)time(NULL));
        flag = 1;
    }

    return ((float)rand() + 1.0)/((float)RAND_MAX + 2.0);
}

//indexの計算
int index(float x,float y)
{
    int index;
    int index_x;
    int index_y;

    index_x = floor(x - map.info.origin.position.x) / map.info.resolution;
    index_y = floor(y - map.info.origin/position.y) / map.info.resolution;
    index   = index_x + index_y * map.info.width;

    return index;
}

int grid_data(float x,float y)
{
    int data = map.data[index(x,y)];

    return data;
}

float get_Yaw(geometry_msgs quaternion q)
{
    double r;
    double p;
    double y;

    tf::Quaternion quat(q.x,q.y,q.z);
    tf::Matrix3x3(quat).getRPY(r,p,y);

    return y
}

//角度差の計算
float angle_diff(float a,float b)
{
    float a_ang = atan2(sin(a),cos(a));
    float b_ang = atan2(sin(b),cos(b));

    float d1 = a-b;
    float d2 = 2*M_PI - fabs(d1);

    if(d1 > 0){
        d2 *= -1.0;
    }
    else if(fabs(d1) < fabs(d2)){
        return d1;
    }
    else{
        return d2;
    }
}

Particle::Particle()
{
    pose.header.frame_id = "map";

    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    quaternionTFToMsg(tf::createQuaternionFromYaw(0),pose.pose.orientation);
    weight = 1 / (double)N;
}

//Particleの初期化
void Particle::p_init(int  width,int  height,float resolution,geometry_msgs::Pose origin)
{
    pose.pose.position.x = random() * width  * resolution + origin.position.x;
    pose.pose.position.y = random() * height * resolution + origin.position.y;
    quaternionTFToMsg(tf::createQuaternionFromYaw(M_PI*(2*random()-1),pose.position.orientation);
}

//Particleの動き更新
void Particle::p_motion_updata(geometry_msgs::PoseStamped current,geometry_msgs::PoseStamped previous)
{
    double dx;
    double dy;
    double dyaw;
    double delta;
    double dist;

    dx   = current.pose.position.x - previous.pose.position.x;
    dy   = current.pose.position.y - previous.pose.position.y;
    dyaw = angle_diff(get_Yaw(current.pose.orientation),get_Yaw(previous.pose.orientation));

    dist = sqrt(dx*dx + dy*dy);

    if(dist < 0.01){
        delta = 0.0;

}

//Particleの尤度の計算
void Particle::p_measurement_updata()
{
}

//Roombaのオドメトリ情報をもとにParticleを移動
void Particle::p_move()
{
    pose.pose.position.x += odometry.pose.pose.position.x * cos(get_Yaw(pose.pose.position.orientation)) - odometry.pose.pose.position.y * sin(get_Yaw(pose.pose.position.orientation));
    pose.pose.position.y += odometry.pose.pose.position.x * sin(get_Yaw(pose.pose.position.orientation)) + odometry.pose.pose.position.y * cos(get_Yaw(pose.pose.position.orientation));
    quaternionTFToMsg(,pose.pose.orientation);

    init_odometry.pose.pose.position.x = odometry.pose.pose.position.x;
    init_odometry/pose.pose.position.y = odometry.pose.pose.position.y;

}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"localizer");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Subscriber map_sub = nh.subscribe("/map",100,map_callback);
    ros::Subscriber lsr_sub = nh.subscribe("/scan",100,laser_callback);
    ros::Subscriber odo_sub = nh.subscribe("/roomba/odometry",100,odometry_callback);

    ros::Publisher pose1_sub = nh.advertise<geometry_msgs::Pose2D>("/chibi20/pose",100);
    ros::Publisher pose2_sub = nh.advertise<geometry_msgs::Pose2D>("/poses",100);

    private_nh.getParam("N",N);
    std::cout << "N: " << N << std::endl;

    ros::Rate rate(10.0);
    while(ros::ok()){
        if(get_map){
    }
    ros::spinOnce();
    rate.sleep();

    return 0;
}
