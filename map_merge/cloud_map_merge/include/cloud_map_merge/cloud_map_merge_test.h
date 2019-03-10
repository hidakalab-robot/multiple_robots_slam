#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose2D.h>
#include <pcl_ros/point_cloud.h>
#include <thread>
#include <mutex>
#include <boost/thread.hpp>
#include <forward_list>

class CloudMapMerge
{
private:
    struct robotInfo{
        std::mutex mutex;
        std::string name;
        geometry_msgs::Pose2D initPose;
        ros::Subscriber mapSub;
        sensor_msgs::PointCloud2 rosMap;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclMap;
        bool initialized = false;
    };

    boost::shared_mutex robotListMutex;
    ros::NodeHandle sub;
    ros::NodeHandle pub;
    ros::NodeHandle param;

    //vectorよりlistの方がいいかも
    //std::vector<robotInfo> robotList;
    std::forward_list<robotInfo> robotList;

    std::string MAP_TOPIC;
    std::string MERGE_MAP_FRAME;
    std::string PARAM_NAMESPACE;
    double CEILING_HEIGHT;
    double FLOOR_HEIGHT;
    ros::Publisher mapPub;

    void robotRegistration(void);//ロボットの情報を登録
    bool isMapTopic(ros::master::TopicInfo& topic);
    void initPoseLoad(CloudMapMerge::robotInfo& info);
    void mapUpdate(const sensor_msgs::PointCloud2::ConstPtr& msg, CloudMapMerge::robotInfo& info);//ロボットごとのマップを更新する
    void mapMerging(void);//subscribeしたmapを合成
    void registrationLoop(void);
    void mergingLoop(void);

public:
    CloudMapMerge();
    ~CloudMapMerge(){};
    void multiThreadMainLoop(void);//登録とマージとマップの更新がマルチスレッドになってるループ
};

CloudMapMerge::CloudMapMerge():param("~"){
    param.param<std::string>("map_topic",MAP_TOPIC,"/rtabmap/cloud_obstacles");
    param.param<std::string>("merge_map_frame",MERGE_MAP_FRAME,"merge_map");
    param.param<std::string>("param_namespace",PARAM_NAMESPACE,"map_merge");
    param.param<double>("ceiling_height",CEILING_HEIGHT,2.4);
    param.param<double>("floor_height",FLOOR_HEIGHT,-0.05);
    mapPub = pub.advertise<sensor_msgs::PointCloud2>("merge_map",1,true);
}

void CloudMapMerge::robotRegistration(void){
    //masterに登録されているtopicを読み込み
    ros::master::V_TopicInfo topicList;
    ros::master::getTopics(topicList);
    
    //topicListの中からmapのトピックのみを抽出
    //for(int i=0;i<topicList.size();++i){
    for(auto& topic : topicList){
        //maptopicであるか確認
        if(!isMapTopic(topic)){
            continue;
        }
        std::string robotName = ros::names::parentNamespace(topic.name);
        //すでに登録されていないかロボットの名前を確認
        {
            bool isRegisterd = false;
            {
                boost::shared_lock<boost::shared_mutex> bLock(robotListMutex);
                //for(int j=0;j<robotList.size();++j){
                for(auto& robot : robotList){
                    std::lock_guard<std::mutex> lock(robot.mutex);
                    if(robotName == robot.name){
                        isRegisterd = true;
                    }
                }
            }
            if(isRegisterd){
                continue;
            }
        }
        //登録されていない場合だけ以下で登録を行う
        {
            std::lock_guard<boost::shared_mutex> bLock(robotListMutex);
            robotList.emplace_front();
            CloudMapMerge::robotInfo& robot = robotList.front();
            {
                std::lock_guard<std::mutex> lock(robot.mutex);
                robot.name = robotName;
                initPoseLoad(robot);
                //robot.mapSub = sub.subscribe<sensor_msgs::PointCloud2>(robot.name+MAP_TOPIC, 1,boost::bind(&CloudMapMerge::mapUpdate,this,_1,robot));
                robot.mapSub = sub.subscribe<sensor_msgs::PointCloud2>(robot.name+MAP_TOPIC, 1, [this, &robot](const sensor_msgs::PointCloud2::ConstPtr& msg) {mapUpdate(msg, robot);});
                robot.initialized = true;
            }
        }
    }
}

bool CloudMapMerge::isMapTopic(ros::master::TopicInfo& topic){
    bool isMap = topic.name == ros::names::append(ros::names::parentNamespace(topic.name),MAP_TOPIC);
    bool isPointCloud = topic.datatype == "sensor_msgs/PointCloud2";
    return isMap && isPointCloud;
}

void CloudMapMerge::initPoseLoad(CloudMapMerge::robotInfo& robot){
    std::string ns = ros::names::append(robot.name,PARAM_NAMESPACE);
    param.param<double>(ros::names::append(ns,"init_pose_x"), robot.initPose.x, 0.0);
    param.param<double>(ros::names::append(ns,"init_pose_y"), robot.initPose.y, 0.0);
    param.param<double>(ros::names::append(ns,"init_pose_yaw"), robot.initPose.theta, 0.0);
}

void CloudMapMerge::mapUpdate(const sensor_msgs::PointCloud2::ConstPtr& msg, CloudMapMerge::robotInfo& robot){
    //timestampが進んでいたらsensor_msgを更新してpclに変換する
    std::lock_guard<std::mutex> lock(robot.mutex);
    if(robot.initialized && msg -> header.stamp > robot.rosMap.header.stamp){
        return;
    }
    robot.rosMap = *msg;
    robot.pclMap = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *robot.pclMap);
    robot.initialized = true;
}

void CloudMapMerge::mapMerging(void){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    {
        boost::shared_lock<boost::shared_mutex> lock(robotListMutex);
        //for(int i=0;i<robotList.size();++i){
        for(auto& robot : robotList){
            Eigen::Matrix2d rotation;
            //std::lock_guard<std::mutex> lock(robotList[i].mutex);
            //rotation << cos(robotList[i].initPose.theta) , -sin(robotList[i].initPose.theta) , sin(robotList[i].initPose.theta) , cos(robotList[i].initPose.theta);

            std::lock_guard<std::mutex> lock(robot.mutex);
            rotation << cos(robot.initPose.theta) , -sin(robot.initPose.theta) , sin(robot.initPose.theta) , cos(robot.initPose.theta);

            Eigen::Vector2d tempPoint;
            Eigen::Vector2d rotatePoint;

            // for(int j=0;j<robotList[i].pclMap->points.size();j++){
            //     if(robotList[i].pclMap->points[j].z < CEILING_HEIGHT && robotList[i].pclMap->points[j].z > FLOOR_HEIGHT){
            //         tempPoint << robotList[i].pclMap->points[j].x , robotList[i].pclMap->points[j].y;
            //         rotatePoint = rotation * tempPoint;
            //         mergeCloud -> points.emplace_back(pcl::PointXYZRGB());
            //         pcl::PointXYZRGB& point = mergeCloud -> points.back();
            //         point = robotList[i].pclMap->points[j];
            //         point.x = rotatePoint.x() + robotList[i].initPose.x;
            //         point.y = rotatePoint.y() + robotList[i].initPose.y;
            //     }
            // }

            // for(int j=0;j<robotList[i].pclMap.points.size();j++){
            //     if(robotList[i].pclMap.points[j].z < CEILING_HEIGHT && robotList[i].pclMap.points[j].z > FLOOR_HEIGHT){
            //         tempPoint << robotList[i].pclMap.points[j].x , robotList[i].pclMap.points[j].y;
            //         rotatePoint = rotation * tempPoint;
            //         mergeCloud -> points.emplace_back(pcl::PointXYZRGB());
            //         pcl::PointXYZRGB& point = mergeCloud -> points.back();
            //         point = robotList[i].pclMap.points[j];
            //         point.x = rotatePoint.x() + robotList[i].initPose.x;
            //         point.y = rotatePoint.y() + robotList[i].initPose.y;
            //     }
            // }

            for(auto& point : robot.pclMap->points){
                if(point.z < CEILING_HEIGHT && point.z > FLOOR_HEIGHT){
                    tempPoint << point.x , point.y;
                    rotatePoint = rotation * tempPoint;
                    mergeCloud -> points.emplace_back(pcl::PointXYZRGB());
                    pcl::PointXYZRGB& insertPoint = mergeCloud -> points.back();
                    insertPoint = point;
                    insertPoint.x = rotatePoint.x() + robot.initPose.x;
                    insertPoint.y = rotatePoint.y() + robot.initPose.y;
                }
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

void CloudMapMerge::registrationLoop(void){
    while(ros::ok()){
        robotRegistration();
    }
}

void CloudMapMerge::mergingLoop(void){
    while(ros::ok()){
        mergingLoop();
    }
}

void CloudMapMerge::multiThreadMainLoop(void){
    ros::spinOnce();
    std::thread mergingThread([this]() { mergingLoop(); });
    std::thread registrationThread([this]() { registrationLoop(); });
    //std::thread mergingThread(&CloudMapMerge::mergingLoop,this);
    //std::thread registrationThread(&CloudMapMerge::registrationLoop,this);
    ros::spin();
    mergingThread.join();//スレッドの終了を待つ??
    registrationThread.join();
}