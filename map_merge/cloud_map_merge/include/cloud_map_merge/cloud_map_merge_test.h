#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <thread>
#include <mutex>
#include <boost/thread.hpp>

class CloudMapMerge
{
private:
    struct robotInfo{
        //std::mutex mutex;
        std::string name;
        double initX;
        double initY;
        double initYaw;
        ros::Subscriber mapSub;
        sensor_msgs::PointCloud2 rosMap;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclMap;
        bool initialized = false;
    };

    boost::shared_mutex robotListMutex;
    ros::NodeHandle sub;
    ros::NodeHandle pub;
    ros::NodeHandle param;
    std::vector<CloudMapMerge::robotInfo> robotList;

    std::string MAP_TOPIC;
    std::string MERGE_MAP_FRAME;
    std::string PARAM_NAMESPACE;
    double CEILING_HEIGHT;
    double FLOOR_HEIGHT;
    ros::Publisher mapPub;

    bool isMapTopic(ros::master::TopicInfo& topic);
    void initPoseLoad(CloudMapMerge::robotInfo& info);
    void mapUpdate(const sensor_msgs::PointCloud2::ConstPtr& msg, CloudMapMerge::robotInfo& info);//ロボットごとのマップを更新する

public:
    CloudMapMerge();
    ~CloudMapMerge(){};

    void robotRegistration(void);//ロボットの情報を登録
    void mapMerging(void);//subscribeしたmapを合成
};

//std::lock_guard<std::mutex> s_lock(subscription.mutex);
//boost::shared_lock<boost::shared_mutex> lock(subscriptions_mutex_);

CloudMapMerge::CloudMapMerge():param("~"){
    param.param<std::string>("map_topic",MAP_TOPIC,"rtabmap/cloud_obstacles");
    param.param<std::string>("merge_map_frame",MERGE_MAP_FRAME,"merge_map");
    param.param<std::string>("param_namespace",PARAM_NAMESPACE,"cloud_map_merge");
    param.param<double>("ceiling_height",CEILING_HEIGHT,2.4);
    param.param<double>("floor_height",FLOOR_HEIGHT,-0.05);
    mapPub = pub.advertise<sensor_msgs::PointCloud2>("merge_map",1,true);
}

void CloudMapMerge::robotRegistration(void){

    //masterに登録されているtopicを読み込み
    ros::master::V_TopicInfo topicList;
    ros::master::getTopics(topicList);
    
    //topicListの中からmapのトピックのみを抽出
    for(int i=0;i<topicList.size();++i){
        //maptopicであるか確認
        if(!isMapTopic(topicList[i]))
        {
            continue;
        }
        std::string robotName = ros::names::parentNamespace(topicList[i].name);
        //すでに登録されていないかロボットの名前を確認
        {
            bool isRegisterd = false;
            for(int j=0;j<robotList.size();++j){
                if(robotName == robotList[j].name)
                {
                    isRegisterd = true;
                }
            }
            if(isRegisterd){
                continue;
            }
        }
        //登録されていない場合だけ以下で登録を行う
        robotList.emplace_back();
        CloudMapMerge::robotInfo& info = robotList.back();
        info.name = robotName;
        initPoseLoad(info);
        info.mapSub = sub.subscribe<sensor_msgs::PointCloud2>(info.name+MAP_TOPIC, 1,boost::bind(&CloudMapMerge::mapUpdate,this,_1,info));
    }
}

bool CloudMapMerge::isMapTopic(ros::master::TopicInfo& topic){
    bool isMap = topic.name == ros::names::append(ros::names::parentNamespace(topic.name),MAP_TOPIC);
    bool isPointCloud = topic.datatype == "sensor_msgs/PointCloud2";
    return isMap && isPointCloud;
}

void CloudMapMerge::initPoseLoad(CloudMapMerge::robotInfo& info){
    std::string ns = ros::names::append(info.name,PARAM_NAMESPACE);
    param.param<double>(ros::names::append(ns,"init_pose_x"), info.initX, 0.0);
    param.param<double>(ros::names::append(ns,"init_pose_y"), info.initY, 0.0);
    param.param<double>(ros::names::append(ns,"init_pose_yaw"), info.initYaw, 0.0);
}

void CloudMapMerge::mapUpdate(const sensor_msgs::PointCloud2::ConstPtr& msg, CloudMapMerge::robotInfo& info){
    //timestampが進んでいたらsensor_msgを更新してpclに変換する
    if(info.initialized && msg -> header.stamp > info.rosMap.header.stamp){
        return;
    }

    info.rosMap = *msg;
    info.pclMap = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *info.pclMap);
    info.initialized = true;
}

void CloudMapMerge::mapMerging(void){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i=0;i<robotList.size();++i){
        Eigen::Matrix2d rotation;
        rotation << cos(robotList[i].initYaw) , -sin(robotList[i].initYaw) , sin(robotList[i].initYaw) , cos(robotList[i].initYaw);

        Eigen::Vector2d tempPoint;
        Eigen::Vector2d rotatePoint;

        for(int j=0;j<robotList[i].pclMap->points.size();j++){
            if(robotList[i].pclMap->points[j].z < CEILING_HEIGHT && robotList[i].pclMap->points[j].z > FLOOR_HEIGHT){
                tempPoint << robotList[i].pclMap->points[j].x , robotList[i].pclMap->points[j].y;
                rotatePoint = rotation * tempPoint;
                mergeCloud -> points.emplace_back(pcl::PointXYZRGB());
                pcl::PointXYZRGB& point = mergeCloud -> points.back();
                point = robotList[i].pclMap->points[j];
                point.x = rotatePoint.x() + robotList[i].initX;
                point.y = rotatePoint.y() + robotList[i].initY;
            }
        }

    }
    mergeCloud -> width = mergeCloud -> points.size();
    mergeCloud -> height = 1;
    mergeCloud -> is_dense = false;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg (*mergeCloud, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = MERGE_MAP_FRAME;
    mapPub.publish(msg);
}