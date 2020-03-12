#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include <math.h>
#include <random>

class Particle
{
public:
    Particle();
    void p_init(float x,float y,float yaw,float sigma_x,float sigma_y,float sigma_yaw);
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

std::vector<Particle> particles

int N;                      //Particleの数
int max_index;
float init_x;               //初期位置x
float init_y;               //初期位置y
float init_yaw;             //初期位置yaw
float init_x_sigama;
float init_y_sigma;
float init_yaw_sigma;
float MAX_Range;
int Range_Step;             //尤度step
float x_sigma = 0.5;        //xの分散
float y_sigma = 0.5;        //yの分散
float yaw_sigma = 0.5;      //yawの分散
float x_TH;                 //xのしきい値
float y_TH;                 //yのしきい値
float yaw_TH;               //yawのしきい値
float p_sigma;              //particleの
float move_noise_x;         //noise
float move_noise_y;         //noise
float move_noise_yaw;       //noise

float judge_distance_value;
float judge_angle_value;

double average_weight;      //尤度の平均
double weight_slow = 0.0;
double weight_fast = 0.0;
double alpha_slow;
double alpha_fast;
bool get_map = false;       //mapを取得したかどうかの判定
bool updata_flag = false;   //更新するかどうかの判定

//メルセンヌツイスタ
std::random_device seed;
std::mt19937 engine(seed());

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

        p.p_init(init_x,init_y,init_yaw,init_x_sigma,init_y_sigma,init_yaw_sigma);

        geometry_msgs::Pose p_pose;
        p_pose.pose.position.x = p.pose.pose.position.x;
        p_pose.pose.position.y = p.pose.pose.position.y;
        p_pose.pose.position.z = 0.0;
        quaternionTFToMsg(tf::createQuaternionFromYaw(get_Yaw(p.pose.pose.orientation)),p_pose.orientation);

        //particlesに格納
        particles.push_back(p);

        //PoseArrayに格納
        poses.poses.push_back(p_pose);
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

/*
// --- 乱数生成 (多分使わない)---
float random()
{
    static int flag;

    if(flag == 0){
        srand((unsigned int)time(NULL));
        flag = 1;
    }

    return ((float)rand() + 1.0)/((float)RAND_MAX + 2.0);
}
*/

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

/*  --- いらない(一応残しとく) ---
//正規分布
float random_normal(float mu,float sigma)
{
    float z = 1/(sqrt(2.0*M_PI)*sigma)*exp(-pow((random()-mu),2)/(2*pow(sigma,2)));

    return z;
}
*/

//Rangeの更新
float get_Range(float x,float y,float yaw)
{
    int cx_0;
    int cy_0;
    int cx_1;
    int cy_1;
    int xstep;
    int ystep;
    int error;
    bool judge = false;

    cx_0 = (int)((x - map.info.origin.position.x)/map.info.resolution);
    cy_0 = (int)((y - map.info.origin.position.y)/map.info.resolution);

    cx_1 = (int)((x + MAX_Range*cos(yaw) - map.info.origin.position.x)/map.info.resolution);
    cy_1 = (int)((y + MAX_Range*sin(yaw) - map.info.origin.position.y)/map.info.resolution);

    if(fabs(cx_1 - cx_0) < fabs(cy_1 - cy_0))
        judge = true;

    if(judge){
        int tmp;
        tmp  = cx_1;
        cx_1 = cy_1;
        cy_1 = tmp;

        tmp  = cx_0;
        cx_0 = cy_0;
        cy_0 = tmp;
    }

    dx = fabs(cx_1 - cx_0);
    dy = fabs(cy_1 - cy_0);

    cx = cx_0;
    cy = cy_0;

    if(cx_1 > cx_0)
        xstep = 1;
    else
        xstep = -1;

    if(cy_1 > cy_0)
        ystep = 1;
    else
        ystep = -1;

    if(judge){
        if(cy < 0 || cy > map.info.width || cx < 0 || cx > map.info.height || map.data[cx*map.info.width + cy] != 0){
            return sqrt(pow((cx - cx_0),2) + pow((cy - cy_0),2)) * map.info.resolution;
        }
    }
    else{
        if(cx < 0 || cx > map.info.width || cy < 0 || cy > map.info.height || map.data[cy*map.info.width + cx] != 0){
            return sqrt(pow((cx - cx_0),2) + pow((cy - cy_0),2)) * map.info.resolution;
        }
    }

    while(cx != (cx_1 + xstep)){
        x += xstep;
        error += dy;
        if(2*error >= dx){
            y += ystep;
            error -= dx;
        }
        if(judge){
            if(cy < 0 || cy > map.info.width || cx < 0 || cx > map.info.height || map.data[cx*map.info.width + cy] != 0){
                return sqrt(pow((cx - cx_0),2) + pow((cy - cy_0),2)) * map.info.resolution;
            }
        }
        else{
            if(cx < 0 || cx > map.info.width || cy < 0 || cy > map.info.height || map.data[cy*map.info.width + cx] != 0){
                return sqrt(pow((cx - cx_0),2) + pow((cy - cy_0),2)) * map.info.resolution;
            }
        }
    }

    return MAX_Range;
}

//更新するかしないかの判定
bool judge_updata(float distance_value,float angle_value)
{
    if(distance_value > judge_distance_value || angle_value > judge_angle_value){
        return true;
    }
    else{
        return false;
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
void Particle::p_init(float x,float y,float yaw,float sigma_x,float sigma_y,float sigma_yaw)
{
    //random()のみでParticleをばらまく
    /*
    pose.pose.position.x = random() * width  * resolution + origin.position.x;
    pose.pose.position.y = random() * height * resolution + origin.position.y;
    quaternionTFToMsg(tf::createQuaternionFromYaw(M_PI*(2*random()-1),pose.position.orientation);
    */

    //正規分布でParticleをばらまく(推定位置を引数)
    std::normal_distribution<> dist_x(x,sigma_x);
    double rand = dist_x(engine);
    if(rand > 0 && rand < map.info.width*map.info.resolution){
        pose.pose.position.x = rand;
    }

    std::normal_distribution<> dist_y(y,sigma_y);
    rand = dist_y(engine);
    if(rand > 0 && rand < map.info.height*map.info.resolution){
        pose.pose.position.y = rand;
    }

    std::normal_distribution<> dist_yaw(yaw,sigma_yaw);
    rand = dist_yaw(engine);
    if(rand > -M_PI && rand < M_PI){
        quaternionTFToMsg(tf::createQuaternionFromYaw(rand),pose.pose.orientation);
    }

}

//Particleの動きを更新
void Particle::p_motion_updata(geometry_msgs::PoseStamped current,geometry_msgs::PoseStamped previous)
{
    float dx;
    float dy;
    float dyaw;
    float distance_sum = 0.0;
    float angle_sum    = 0.0;

    //current - previous (移動量)
    dx   = current.pose.position.x - previous.pose.position.x;
    dy   = current.pose.position.y - previous.pose.position.y;
    dyaw = angle_diff(get_Yaw(current.pose.orientation),get_Yaw(previous.pose.orientation));

    distance_sum += sqrt(dx*dx + dy*dy);
    angle_sum += fabs(dyaw);

    updata_flag = judge_updata(distance_sum,angle_sum);

    //particleを移動
    for(int i = 0; i < particles.size(); i++){
        particles[i].p_move(dx,dy,dyaw);
    }
}

//Particleの尤度の計算
void Particle::p_measurement_updata()
{
    double angle;
    double map_range;
    double range_diff = 0.0;
    double p_weight_sum = 0.0;

    //更新したら尤度を計算する
    if(updata_flag){
        for(int i = 0; i < N; i++){
            for(int j = 0; j < laser.ranges.size(); j += Range_Step){
                angle = laser.angle_min + j*laser.angle_increment;
                map_range = (double)(get_Range(particles[i].pose.pose.position.x,particles[i].pose.pose.position.y,get_Yaw(particles[i].pose.pose.orientation)+angle));

                rand_diff += pow((laser.ranges[j] - map_range),2);
            }
            particles[i].weight = exp(-rand_diff/(2*pow(p_sigma,2)));
            p_weight_sum += particles[i].weight;
        }

        for(int i = 0; i < N; i++){
            particles[i].weight /= p_weight_sum;
        }

        average_weight = p_weight_sum / (double)N;
    }
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

    //Publisher(lmcとgppへpublish)
    ros::Publisher lmc_sub = nh.advertise<geometry_msgs::Pose2D>("/chibi20/pose",100);
    ros::Publisher gpp_sub = nh.advertise<geometry_msgs::Pose2D>("/poses",100);

    private_nh.getParam("N",N);
    private_nh.getParam("init_x",init_x);
    private_nh.getParam("init_y",init_y);
    private_nh.getParam("init_yaw",init_yaw);
    private_nh.getParam("init_x_sigma",init_x_sigma);
    private_nh.getParam("init_y_sigma",init_y_sigma);
    private_nh.getParam("init_yaw_sigma",init_yaw_sigma);
    private_nh.getParam("p_sigma",p_sigma);
    private_nh.getParam("MAX_Range",MAX_Range);
    private_nh.getParam("Range_Step",Range_Step);

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    tf::StampedTransform temp_tf_stamped;
    temp_tf_stamped = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(0.0),tf::Vector3(0.0,0.0,0.0)),ros::Time::now(),"map","odom");

    //位置の初期化
    current_pose.pose.position.x = 0.0;
    current_pose.pose.position.y = 0.0;
    quaternionTFToMsg(tf::createQuaternionFromYaw(0.0),current.pose.orientation);
    previous_pose  = current_pose;

    ros::Rate rate(10.0);
    while(ros::ok()){
        if(get_map && !laser.ranges.empty()){
            estimated_pose.header.frame_id = "map";

            tf::StampedTransform transform;
            transform = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(0.0),tf::Vector3(0.0,0.0,0.0)),ros::Time::now(),"odom","base_link");
            try{
                //base_linkからodomへ
                listener.waitForTransform("odom","base_link",ros::Time(0),ros::Duration());
                listener.lookupTransform("odom","base_link",ros::Time(0),transform);
            }
            catch(tf::TransformException &ex){
                //見つからなかった場合
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }

            //current_poseへ格納
            current_pose.pose.position.x = transform.getOrigin().x();
            current_pose.pose.position.y = transform.getOrigin().y();
            quaternionTFToMsg(transform.getRotation(),current_pose.pose.orientation);

            //Particleをばらまく
            if(x_sigma < x_TH || y_sigma < y_TH || yaw_sigma < yaw_TH){
                x_sigma   = 0.3;
                y_sigma   = 0.3;
                yaw_sigma = 0.3;

                std::vector<Particle> set_particles;
                for(int i = 0; i < N; i++){
                    Particle p;

                    p.p_init(estimated_pose.pose.position.x,estimated_pose.pose.position.y,get_Yaw(estimated_pose.pose.position.yaw),x_sigma,y_sigma,yaw_sigma);
                    set_particles.push_back(p);
                }
                particles = set_particles;
            }

            //Particleの動きを更新
            particles.p_motion_updata(current_pose,previous_pose);


            //尤度処理
            particles.p_measurement_updata();

            for(int i = 0; i < N; i++){
                if(particles[i].weight > particles[max_index].weight){
                    max_index = i;
                }
            }

            if(average_weight = 0 || std::isnan(average_weight)){
                average_weight = 1 / (double)N;
                weight_slow = weight_fast = weight_average;
            }

            if(weight_slow == 0){
                weight_slow = weight_average;
            }
            else{
                weight_slow += alpha*(weight_average - weight_slow);
            }

            if(weight_fast == 0){
                weight_fast = weight_average;
            }
            else{
                weight_fast += alpha*(weight_average - weight_fast)
            }

            //Resampling
            if(updata_flag){
                std::uniform_real_distribution<> dist(0.0,0.1)
                int index = (int)(dist(engine)*N);
                double beta = 0.0;
                double mv = particles[max_index].weight;
                std::vector<Particle> new_particles;

                double w;
                if((1-weight_fast/weight_slow) > 0){
                    w = 1 - weight_fast/weight_slow;
                }
                else{
                    w = 0.0;
                }
                for(int i = 0; i < N; i++){
                    if(w < dist(engine)){
                        beta += dist(engine) * 2.0 * mv;
                        while(beta > particles[index].weight){
                            beta -= particles[index].weight;
                            index -= (index+1)%N;
                        }
                        new_particles.push_back(particles[index]);
                    }
                    else{
                        Particle p;
                        p.p_init(estimated_pose.pose.position.x,estimated_pose.pose.position.y,get_Yaw(estimated_pose.pose.orientation),init_x_sigma,init_y_sigma,init_yaw_sigma);
                        new_particles.push_back(p);
                    }
                }

                particles = new_particles;
            }

            try{
                updata_flag = false;

                //尤度が一番大きいparticleを算出
                estimated_pose.header.frame_id = "map";
                estimated_pose.pose = particles[max_index].pose.pose;

                //estimated_poseをestimated_pose2dへ
                geometry_msgs::Pose2D estimated_pose2d;

                estimated_pose2d.x     = estimated_pose.pose.position.x;
                estimated_pose2d.y     = estimated_pose.pose.position.y;
                estimated_pose2d.theta = get_Yaw(estimated_pose.pose.orientation);
                lmc_sub.publish(estimated_pose2d);
                gpp_sub.publish(estimated_pose2d);

                tf::StampedTransform map_transform;
                map_transform.setOrigin(tf::Vector3(estimated_pose.pose.position.x,estimated_pose.pose.position.y,0.0));
                map_transform.setRotation(tf::createQuaternionFromYaw(get_Yaw(estimated_pose.pose.orientation)));
                tf::Stamped<tf::Pose> tf_stamped(map_transform.inverse(),laser.header.stamp,"base_link");
                tf::Stamped<tf::Pose> odom_to_map;
                listener.transformPose("odom",tf_stamped,odom_to_map);
                tf::Transform latest_tf = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),tf::Point(odom_to_map.getOrigin()));
                temp_tf_stamped = tf::StampedTransform(latest_tf.inverse(),laser.header.stamp,"map","odom");
                broadcaster.sendTransform(temp_tf_stamped);

            }
            catch(tf::TransformException ex){
                ROS_ERROR("ERROR!");
                ROS_ERROR("%s", ex.what());
            }
        }
        ros::spinOnce();
        rate.sleep();

        previous_pose = current_pose;
    }
    return 0;
}
