//
//Coderview後,localizer.cppを改良(コアダンプする)
//
#include "localizer/localizer.h"

//乱数生成
std::random_device seed;
std::mt19937 engine(seed());

Particle::Particle() :private_nh("~")
{
    //parameter
    private_nh.getParam("N",N);
    private_nh.getParam("INIT_X",INIT_X);
    private_nh.getParam("INIT_Y",INIT_Y);
    private_nh.getParam("INIT_YAW",INIT_YAW);
    private_nh.getParam("INIT_X_COV",INIT_X_COV);
    private_nh.getParam("INIT_Y_COV",INIT_Y_COV);
    private_nh.getParam("INIT_YAW_COV",INIT_YAW_COV);
    private_nh.getParam("MAX_RANGE",MAX_RANGE);
    private_nh.getParam("RANGE_STEP",RANGE_STEP);
    private_nh.getParam("X_COV_TH",X_COV_TH);
    private_nh.getParam("Y_COV_TH",Y_COV_TH);
    private_nh.getParam("P_COV",P_COV);
    private_nh.getParam("MOVE_NOISE_1",MOVE_NOISE_1);
    private_nh.getParam("MOVE_NOISE_2",MOVE_NOISE_2);
    private_nh.getParam("MOVE_NOISE_3",MOVE_NOISE_3);
    private_nh.getParam("MOVE_NOISE_4",MOVE_NOISE_4);
    private_nh.getParam("ALPHA_SLOW",ALPHA_SLOW);
    private_nh.getParam("ALPHA_FAST",ALPHA_FAST);
    private_nh.getParam("JUDGE_DISTANCE_VALUE",JUDGE_DISTANCE_VALUE);
    private_nh.getParam("JUDGE_ANGLE_VALUE",JUDGE_ANGLE_VALUE);

    private_nh.param("Hz",Hz,{10});
    private_nh.param("x_cov",x_cov,{0.5});
    private_nh.param("y_cov",y_cov,{0.5});
    private_nh.param("yaw_cov",yaw_cov,{0.5});
    private_nh.param("weight_slow",weight_slow,{0.0});
    private_nh.param("weight_fast",weight_fast,{0.0});
    private_nh.param("have_map",have_map,{false});
    private_nh.param("is_update",is_update,{false});

    //subscriber
    ros::Subscriber map_sub = nh.subscribe("/fixed_map",100,&Particle::map_callback,this);
    ros::Subscriber lsr_sub = nh.subscribe("/scan",100,&Particle::laser_callback,this);
    ros::Subscriber odo_sub = nh.subscribe("/roomba/odometry",100,&Particle::odometry_callback,this);

    //Publisher
    ros::Publisher estimated_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("estimated_pose",100);

    pose.header.frame_id = "map";

    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    quaternionTFToMsg(tf::createQuaternionFromYaw(0),pose.pose.orientation);
    weight = 1 / (double)N;
}

//Particleの初期化
void Particle::p_init(double x,double y,double yaw,double cov_x,double cov_y,double cov_yaw)
{
    //正規分布でParticleをばらまく
    do{
        std::normal_distribution<> dist_x(x,cov_x);
        pose.pose.position.x = dist_x(engine);

        std::normal_distribution<> dist_y(y,cov_y);
        pose.pose.position.y = dist_y(engine);

        std::normal_distribution<> dist_yaw(yaw,cov_yaw);
        quaternionTFToMsg(tf::createQuaternionFromYaw(dist_yaw(engine)),pose.pose.orientation);
    }while(grid_data(pose.pose.position.x,pose.pose.position.y) != 0);
}

//Particleの動きを更新
void Particle::p_motion_update(geometry_msgs::PoseStamped current,geometry_msgs::PoseStamped previous)
{
    static double distance_sum = 0.0;
    static double angle_sum = 0.0;

    //current - previous (移動量)
    double dx   = current.pose.position.x - previous.pose.position.x;
    double dy   = current.pose.position.y - previous.pose.position.y;
    double dyaw = angle_diff(get_Yaw(current.pose.orientation),get_Yaw(previous.pose.orientation));

    distance_sum += sqrt(dx*dx + dy*dy);
    angle_sum += fabs(dyaw);

    is_update = judge_update(distance_sum,angle_sum);
    if(is_update){
        distance_sum = 0.0;
        angle_sum = 0.0;
    }

    //particleを移動
    p_move(dx,dy,dyaw);
}

//Particleを(dx,dy,dyaw)移動させる
void Particle::p_move(double dx,double dy,double dyaw)
{
    double delta_rot1;
    double yaw = get_Yaw(pose.pose.orientation);

    if(sqrt(dx*dx + dy*dy) < 0.01)
        delta_rot1 = 0;
    else
        delta_rot1 = dyaw;

    double delta_trans = sqrt(dx*dx +dy*dy);
    double delta_rot2  = angle_diff(dyaw,delta_rot1);

    double delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1,0.0)),fabs(angle_diff(delta_rot1,M_PI)));
    double delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2,0.0)),fabs(angle_diff(delta_rot2,M_PI)));

    std::normal_distribution<> dist_rot1(0.0,(MOVE_NOISE_1*delta_rot1_noise*delta_rot1_noise - MOVE_NOISE_2*delta_trans*delta_trans));
    std::normal_distribution<> dist_rot2(0.0,(MOVE_NOISE_1*delta_rot2_noise*delta_rot2_noise - MOVE_NOISE_2*delta_trans*delta_trans));
    std::normal_distribution<> dist_trans(0.0,(MOVE_NOISE_3*delta_trans*delta_trans + MOVE_NOISE_4*delta_rot1_noise*delta_rot1_noise + MOVE_NOISE_4*delta_rot2_noise*delta_rot2_noise));

    double delta_rot1_hat  = angle_diff(delta_rot1,dist_rot1(engine));
    double delta_rot2_hat  = angle_diff(delta_rot2,dist_rot2(engine));
    double delta_trans_hat = delta_trans - dist_trans(engine);

    pose.pose.position.x += delta_trans_hat * cos(yaw + delta_rot1_hat);
    pose.pose.position.y += delta_trans_hat * sin(yaw + delta_rot2_hat);
    quaternionTFToMsg(tf::createQuaternionFromYaw(yaw + delta_rot1_hat + delta_rot2_hat),pose.pose.orientation);
}

//Particleの尤度の計算
void Particle::p_measurement_update()
{
    double angle;
    double map_range;
    double range_diff = 0.0;

    for(int i = 0; i < (int)laser.ranges.size(); i += RANGE_STEP){
        angle = laser.angle_min + i*laser.angle_increment;
        map_range = get_Range(pose.pose.position.x,pose.pose.position.y,get_Yaw(pose.pose.orientation)+ angle);

        range_diff += pow((laser.ranges[i] - map_range),2);
        weight = exp(-range_diff/(2*pow(P_COV,2)));
    }
}

//マップの受取とParticleの初期化
void Particle::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map = *msg;
    have_map = true;

    std::cout << "Get a map" << std::endl;

    for(int i = 0; i < N; i++){
        Particle p;

        p.p_init(INIT_X,INIT_Y,INIT_YAW,INIT_X_COV,INIT_Y_COV,INIT_YAW_COV);
        //particlesに格納
        particles.push_back(p);
    }
    std::cout << "finish mapstep" << std::endl;
}

//レーザの受取
void Particle::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser = *msg;
}

//オドメトリの受取
void Particle::odometry_callback(const nav_msgs::Odometry::ConstPtr& odo)
{
    odometry = *odo;
}

//indexの算出
int Particle::index(double x,double y)
{
    int index_x = floor(x - map.info.origin.position.x) / map.info.resolution;
    int index_y = floor(y - map.info.origin.position.y) / map.info.resolution;
    return index_x + index_y*map.info.width;
}

//map.dataの算出
int Particle::grid_data(double x,double y)
{
    return map.data[index(x,y)];
}

//Yaw算出
double Particle::get_Yaw(geometry_msgs::Quaternion q)
{
    double r, p, y;

    tf::Quaternion quat(q.x,q.y,q.z,q.w);
    tf::Matrix3x3(quat).getRPY(r,p,y);

    return y;
}

//角度差の算出
double Particle::angle_diff(double a,double b)
{
    double a_angle = atan2(sin(a),cos(a));
    double b_angle = atan2(sin(b),cos(b));

    double d1 = a_angle-b_angle;
    double d2 = 2*M_PI - fabs(d1);

    if(d1 > 0)
        d2 *= -1;
    if(fabs(d1) < fabs(d2))
        return d1;
    else
        return d2;
}

//Range(particleの存在範囲)の更新
double Particle::get_Range(double x,double y,double yaw)
{
    int xstep, ystep;

    bool judge = false;

    int cx_0 = (x - map.info.origin.position.x)/map.info.resolution;
    int cy_0 = (y - map.info.origin.position.y)/map.info.resolution;

    int cx_1 = (x + MAX_RANGE*cos(yaw) - map.info.origin.position.x)/map.info.resolution;
    int cy_1 = (y + MAX_RANGE*sin(yaw) - map.info.origin.position.y)/map.info.resolution;

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

    int dx = fabs(cx_1 - cx_0);
    int dy = fabs(cy_1 - cy_0);

    int cx = cx_0;
    int cy = cy_0;
    int error = 0;

    if(cx_1 > cx_0)
        xstep = 1;
    else
        xstep = -1;

    if(cy_1 > cy_0)
        ystep = 1;
    else
        ystep = -1;

    if(judge){
        if(cy < 0 || cy > (int)map.info.width || cx < 0 || cx > (int)map.info.height || map.data[cx*(int)map.info.width + cy] != 0){
            return sqrt(pow((cx - cx_0),2) + pow((cy - cy_0),2)) * map.info.resolution;
        }
    }
    else{
        if(cx < 0 || cx > (int)map.info.width || cy < 0 || cy > (int)map.info.height || map.data[cy*(int)map.info.width + cx] != 0){
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
            if(cy < 0 || cy > (int)map.info.width || cx < 0 || cx > (int)map.info.height || map.data[cx*(int)map.info.width + cy] != 0){
                return sqrt(pow((cx - cx_0),2) + pow((cy - cy_0),2)) * map.info.resolution;
            }
        }
        else{
            if(cx < 0 || cx > (int)map.info.width || cy < 0 || cy > (int)map.info.height || map.data[cy*(int)map.info.width + cx] != 0){
                return sqrt(pow((cx - cx_0),2) + pow((cy - cy_0),2)) * map.info.resolution;
            }
        }
    }

    return MAX_RANGE;
}

//更新するかしないかの判定
bool Particle::judge_update(double distance_value,double angle_value)
{
    if(distance_value > JUDGE_DISTANCE_VALUE || angle_value > JUDGE_ANGLE_VALUE)
        return true;
    else
        return false;

}

//新しい分散の算出
void Particle::create_new_cov(double* cov_1,double* cov_2,double* cov_3)
{
    double ave_1 = 0.0;
    double ave_2 = 0.0;
    double ave_3=  0.0;

    for(int i = 0; i < N; i++){
        ave_1 += particles[i].pose.pose.position.x;
        ave_2 += particles[i].pose.pose.position.y;
        ave_3 += get_Yaw(particles[i].pose.pose.orientation);
    }

    ave_1 /= N;
    ave_2 /= N;
    ave_3 /= N;

    double new_cov_1 = 0.0;
    double new_cov_2 = 0.0;
    double new_cov_3 = 0.0;

    //新しい偏差を計算
    for(int i = 0; i < N; i++){
        new_cov_1 += pow((particles[i].pose.pose.position.x - ave_1),2);
        new_cov_2 += pow((particles[i].pose.pose.position.y - ave_2),2);
        new_cov_3 += pow((get_Yaw(particles[i].pose.pose.orientation) - ave_3),2);
    }

    *cov_1 = sqrt(new_cov_1/N);
    *cov_2 = sqrt(new_cov_2/N);
    *cov_3 = sqrt(new_cov_3/N);
}

void Particle::process()
{
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    tf::StampedTransform temp_tf_stamped;
    temp_tf_stamped = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(0.0),tf::Vector3(0.0,0.0,0.0)),ros::Time::now(),"map","odom");

    //位置の初期化
    current_pose.pose.position.x = 0.0;
    current_pose.pose.position.y = 0.0;
    quaternionTFToMsg(tf::createQuaternionFromYaw(0.0),current_pose.pose.orientation);
    previous_pose  = current_pose;

    particles.reserve(1000);

    ros::Rate rate(10);
    while(ros::ok()){
        if(have_map /* && !laser.ranges.empty() */){
            estimated_pose.header.frame_id = "map";

            tf::StampedTransform transform;
            transform = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(0.0),tf::Vector3(0.0,0.0,0.0)),ros::Time::now(),"odom","base_link");
            try{
                //base_linkからodomへ
                listener.waitForTransform("odom","base_link",ros::Time(0),ros::Duration(1.0));
                listener.lookupTransform("odom","base_link",ros::Time(0),transform);
            }
            catch(tf::TransformException &ex){
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }

            //current_poseへ格納
            current_pose.pose.position.x = transform.getOrigin().x();
            current_pose.pose.position.y = transform.getOrigin().y();
            quaternionTFToMsg(transform.getRotation(),current_pose.pose.orientation);

            //Particleをばらまく
            if(x_cov < X_COV_TH || y_cov < Y_COV_TH || yaw_cov < YAW_COV_TH){
                x_cov   = 0.3;
                y_cov   = 0.3;
                yaw_cov = 0.3;

                std::vector<Particle> set_particles;
                for(int i = 0; i < N; i++){
                    Particle p;

                    p.p_init(estimated_pose.pose.position.x,estimated_pose.pose.position.y,get_Yaw(estimated_pose.pose.orientation),x_cov,y_cov,yaw_cov);
                    set_particles.push_back(p);
                }
                particles = set_particles;
            }


            //Particleの動きを更新
            for(int i = 0; i < N; i++){
                particles[i].p_motion_update(current_pose,previous_pose);
            }


            //尤度処理
            double weight_sum = 0.0;
            for(int i = 0; i < N; i++){
                particles[i].p_measurement_update();
                weight_sum += particles[i].weight;
            }
            double weight_average = 0.0;
            int max_index = 0;
            for(int i = 0; i < N; i++){
                weight_average += particles[i].weight;
                if(particles[i].weight > particles[max_index].weight){
                    max_index = i;
                }
                particles[i].weight /= weight_sum;
            }


            weight_average /= (double)N;
            if(weight_average ==  0 || std::isnan(weight_average)){
                weight_average = 1 / (double)N;
                weight_slow = weight_fast = weight_average;
            }

            if(weight_slow == 0.0){
                weight_slow = weight_average;
            }
            else{
                weight_slow += ALPHA_SLOW*(weight_average - weight_slow);
            }

            if(weight_fast == 0.0){
                weight_fast = weight_average;
            }
            else{
                weight_fast += ALPHA_FAST*(weight_average - weight_fast);
            }


            if(is_update){
                //Resampling
                std::uniform_real_distribution<> dist(0.0,1.0);
                int index = (int)(dist(engine)*N);
                double beta = 0.0;
                double mv   = particles[max_index].weight;
                std::vector<Particle> new_particles;

                double w;
                if((1-weight_fast/weight_slow) > 0)
                    w = 1 - weight_fast/weight_slow;
                else
                    w = 0.0;

                for(int i = 0; i < N; i++){
                    if(w < dist(engine)){
                        beta += dist(engine) * 2.0 * mv;
                        while(beta > particles[index].weight){
                            beta -= particles[index].weight;
                            index = (index+1)%N;
                        }
                        new_particles.push_back(particles[index]);
                    }
                    else{
                        Particle p;
                        p.p_init(estimated_pose.pose.position.x,estimated_pose.pose.position.y,get_Yaw(estimated_pose.pose.orientation),INIT_X,INIT_Y,INIT_YAW);
                        new_particles.push_back(p);
                    }
                }

                particles = new_particles;

                create_new_cov(&x_cov,&y_cov,&yaw_cov);
            }
            is_update = false;

        }

        int max_index = 0;
        for(int i = 0; i < N; i++){
            if(particles[i].weight > particles[max_index].weight)
                max_index = i;
        }

        estimated_pose.pose = particles[max_index].pose.pose;

        std::cout << "estimated_pose.x  : " << estimated_pose.pose.position.x << std::endl;
        std::cout << "estimated_pose.y  : " << estimated_pose.pose.position.y << std::endl;
        std::cout << "estimated_pose.yaw: " << get_Yaw(estimated_pose.pose.orientation) << std::endl;

        estimated_pose_pub.publish(estimated_pose);

        try{
            tf::StampedTransform map_transform;
            map_transform.setOrigin(tf::Vector3(estimated_pose.pose.position.x,estimated_pose.pose.position.y,0.0));
            map_transform.setRotation(tf::Quaternion(0,0,get_Yaw(estimated_pose.pose.orientation),1));
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

        ros::spinOnce();
        rate.sleep();

        previous_pose = current_pose;
    }
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"localizer2");
    ROS_INFO("Start Localizer\n");
    Particle particle;
    particle.process();
    return 0;
}

