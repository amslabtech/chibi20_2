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
sensor_msgs::LaserScan laser;
geometry_msgs::PoseStamped estimated_pose;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped previous_pose;
geometry_msgs::Pose2D pose2d;
geometry_msgs::PoseArray poses;

std::vector<Particle> particle

int N;                      //Particleの数
int center_number = 540;
int alpha = 5;
float theta;                //Yaw
float x_sigma;
float y_sigma;
float yaw_sigma;
float move_noise_x;         //noise
float move_noise_y;         //noise
float move_noise_yaw;       //noise
double average_length;      //laserの長さ
double l_length[10];
bool get_map = false;       //mapを取得したかどうかの判定

const float judge_value = 0.01;

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    map = *map;


    /*
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
    */

    for(int i = 0; i < N; i++){
        Particle p;
        do{
            p.init(0.0,0.0,0.0);
        }
        while(grid_data[p.pose.pose.position.x,p.pose.pose.position.y] != 0){
            particle.push_back(p);
        }
        //PoseArrayに格納
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

//Yaw取得
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

/正規分布
float random_normal(float mu,float sigma)
{
    float z = 1/(sqrt(2.0*M_PI)*sigma)*exp(-pow((random()-mu),2)/(2*pow(sigma,2)));

    return z;
}

//LiDARがデータを取得することのできる範囲
float L_range()
{
    return;
}

//Particleの距離計算(何かLandmark見つける)
double p_dist(float x,float y)
{

    return;
}

//更新するかしないかの判定
bool judge_updata(geometry_msgs::PoseStamped current,geometry_msgs::PoseStamped previous)
{
    //judge_valueより移動していなかったら更新しない
    if(sqrt(pow(current.pose.position.x - previous.pose.position.x,2) + pow(current.pose.position.y - previous.pose.position.y)) < judge_value){
        return false;
    }
    else{
        return true;
    }
}

Particle::Particle()
{
    pose.header.frame_id = "map";

    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    quaternionTFToMsg(tf::createQuaternionFromYaw(0),pose.pose.orientation);
    weight = 1 / (float)N;
}

//Particleの初期化
void Particle::p_init(float x,float y,float yaw)
{
    //random()のみでParticleをばらまく
    /*
    pose.pose.position.x = random() * width  * resolution + origin.position.x;
    pose.pose.position.y = random() * height * resolution + origin.position.y;
    quaternionTFToMsg(tf::createQuaternionFromYaw(M_PI*(2*random()-1),pose.position.orientation);
    */

    //正規分布でParticleをばらまく(推定位置を引数)
    pose.pose.position.x = random_normal(x,x_sigma);
    pose.pose.position.y = random_normal(y,y_sigma);
    quaternionTFToMsg(tf::createQuaternionFromYaw(random_normal(yaw    ,yaw_sigma),pose.pose.orientation);

}

//Particleの動きを更新
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

}

//Particleの尤度の計算
void Particle::p_measurement_updata()
{
    if(!laser.ranges.empty()){
        average_length = 0.0;
        for(int i = center_number - alpha; i < center_nunber + alpha; i++){
            average_length += laser.ranges[i];
        }
        average_length /= 2*alpha;

        //Particleの情報と比較して尤度を算出
        /*  --- insert code ---
         */

}

//Particleを(x,y,yaw)へ移動
void Particle::p_move(float x,float y,float yaw)
{
    pose.pose.position.x += x * cos(get_Yaw(pose.pose.position.orientation)) - y * sin(get_Yaw(pose.pose.position.orientation));
    pose.pose.position.y += x * sin(get_Yaw(pose.pose.position.orientation)) + y * cos(get_Yaw(pose.pose.position.orientation));
    quaternionTFToMsg(tf::createQuaternionFromYaw(get_yaw(pose.pose.orientation)),pose.pose.orientation);
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"localizer");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    //Subscriber
    ros::Subscriber map_sub = nh.subscribe("/map",100,map_callback);
    ros::Subscriber lsr_sub = nh.subscribe("/scan",100,laser_callback);
    ros::Subscriber odo_sub = nh.subscribe("/roomba/odometry",100,odometry_callback);

    //Publisher
    ros::Publisher pose1_sub = nh.advertise<geometry_msgs::Pose2D>("/chibi20/pose",100);
    ros::Publisher pose2_sub = nh.advertise<geometry_msgs::Pose2D>("/poses",100);

    private_nh.getParam("N",N);

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    tf::StampedTransform temp_tf_stamped;
    temp_tf_stamped = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(0.0),tf::Vector3(0.0,0.0,0.0)),ros::Time::now(),"map","odom");

    //位置の初期化
    current_pose.pose.position.x = 0.0;
    current_pose.pose.position.y = 0.0;
    quaternionTFToMsg(tf::createQuaternionFromYaw(0.0),current.pose.orientation);
    estimated_pose = current_pose;
    previous_pose  = current_pose;

    ros::Rate rate(10.0);
    while(ros::ok()){
        if(get_map){
            tf::StampedTransform trans;
            transform = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(0.0),tf::Vector3(0.0,0.0,0.0)),ros::Time::now(),"odom","base_link");
            try{
                //base_linkからodomへ
                listener.waitForTransform("odom","base_link",ros::Time(0),ros::Duration());
                listener.lookupTransform("odom","base_link",ros::Time(0),trans);
            }
            catch(tf::TransformException &ex){
                //見つからなかった場合
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }

            //current_poseへ格納
            current_pose.pose.position.x = trans.getOrigin().x();
            current_pose.pose.position.y = trans.getOrigin().y();
            quaternionTFToMsg(trans.getRotation(),current_pose.pose.orientation);

            //Particleをばらまく
            /*
                --- insert code ---
            */

            //Particleの動きを更新
            /*
                --- insert code ---
             */

            //尤度をResamplingした結果から位置推定(estimated_pose)
            /*
                --- insert code ---
            */

            //estimated_pose(Pose) >> Pose2D
            /*
                --- insert code ---
             */

            //Pose2DでPublish
            /*
                --- insert code ---
                pose2d.x     =;
                pose2d.y     =;
                pose2d.theta =;

                pose1_sub.publish(pose2d);
                pose2_sub.publish(pose2d);
             */

    }
    ros::spinOnce();
    rate.sleep();

    return 0;
}
