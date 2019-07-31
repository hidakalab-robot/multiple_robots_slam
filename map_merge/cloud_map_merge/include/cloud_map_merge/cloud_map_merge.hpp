#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose2D.h>
#include <pcl_ros/point_cloud.h>
#include <thread>
#include <mutex>
#include <boost/thread.hpp>
#include <forward_list>
#include <iterator>
#include <exploration_libraly/struct.hpp>
#include <exploration_libraly/constructor.hpp>

class CloudMapMerge
{
private:
    struct robotInfo{
        std::mutex mutex;
        std::string name;
        geometry_msgs::Pose2D initPose;
        ros::Subscriber sub;
        sensor_msgs::PointCloud2 rosMap;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclMap;
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr processedPclMap;
        bool initialized;
        bool update;
    };

    boost::shared_mutex robotListMutex;
    ros::NodeHandle s;
    ros::NodeHandle p;

    ExpLib::pubStruct<sensor_msgs::PointCloud2> pc2_;

    std::forward_list<robotInfo> robotList;

    std::string MAP_TOPIC;
    std::string MERGE_MAP_FRAME;
    std::string PARAM_NAMESPACE;
    double CEILING_HEIGHT;
    double FLOOR_HEIGHT;

    double RAGISTRATION_RATE;
    double MERGING_RATE;

    void robotRegistration(void);//ロボットの情報を登録
    bool isMapTopic(const ros::master::TopicInfo& topic);
    std::string robotNameFromTopicName(const std::string& topicName);
    void initPoseLoad(CloudMapMerge::robotInfo& info);
    void mapUpdate(const sensor_msgs::PointCloud2::ConstPtr& msg, CloudMapMerge::robotInfo& info);//ロボットごとのマップを更新する
    void mapMerging(void);//subscribeしたmapを合成
    void registrationLoop(void);
    void mergingLoop(void);

public:
    CloudMapMerge();
    void multiThreadMainLoop(void);//登録とマージとマップの更新がマルチスレッドになってるループ
};

CloudMapMerge::CloudMapMerge():p("~"),pc2_("merge_map",1,true){
    p.param<std::string>("map_topic",MAP_TOPIC,"/rtabmap/cloud_obstacles");
    p.param<std::string>("merge_map_frame",MERGE_MAP_FRAME,"merge_map");
    p.param<std::string>("param_namespace",PARAM_NAMESPACE,"map_merge");
    p.param<double>("ceiling_height",CEILING_HEIGHT,2.4);
    p.param<double>("floor_height",FLOOR_HEIGHT,-0.05);
    p.param<double>("ragistration_rate", RAGISTRATION_RATE, 0.5);
    p.param<double>("merging_rate", MERGING_RATE, 1.0);
}

void CloudMapMerge::robotRegistration(void){
    //masterに登録されているtopicを読み込み
    ros::master::V_TopicInfo topicList;
    ros::master::getTopics(topicList);
    ROS_INFO_STREAM("registrationThread << robotRegistration\n");
    //topicListの中からmapのトピックのみを抽出
    for(const auto& topic : topicList){
        //maptopicであるか確認
        if(!isMapTopic(topic)){
            continue;
        }
        
        std::string robotName = robotNameFromTopicName(topic.name);
        //すでに登録されていないかロボットの名前を確認
        {
            bool isRegisterd = false;
            {
                boost::shared_lock<boost::shared_mutex> bLock(robotListMutex);
                for(auto&& robot : robotList){
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
        ROS_DEBUG_STREAM("registrationThread << add to system : " << robotName << "\n");
        {
            std::lock_guard<boost::shared_mutex> bLock(robotListMutex);
            robotList.emplace_front();
            CloudMapMerge::robotInfo& robot = robotList.front();
            {
                ROS_DEBUG_STREAM("registrationThread << edit list\n");
                std::lock_guard<std::mutex> lock(robot.mutex);
                robot.name = robotName;
                initPoseLoad(robot);
                robot.sub = s.subscribe<sensor_msgs::PointCloud2>(robot.name+MAP_TOPIC, 1, [this, &robot](const sensor_msgs::PointCloud2::ConstPtr& msg) {mapUpdate(msg, robot);});
                robot.initialized = false;
                robot.update = false;
            }
        }
    }
}

bool CloudMapMerge::isMapTopic(const ros::master::TopicInfo& topic){
    bool isMap = topic.name == ros::names::append(robotNameFromTopicName(topic.name),MAP_TOPIC);
    bool isPointCloud = topic.datatype == "sensor_msgs/PointCloud2";
    return isMap && isPointCloud;
}

std::string CloudMapMerge::robotNameFromTopicName(const std::string& topicName){
    //ros::names::parentNamespace(std::string);はトピック名からネームスペースの文字列を返す
    //例
    ///robot1/map -> /robot1
    ///robot1/abc/map -> /robot1/abc
    return ros::names::parentNamespace(ros::names::parentNamespace(topicName));
}

void CloudMapMerge::initPoseLoad(CloudMapMerge::robotInfo& robot){
    std::string ns = ros::names::append(robot.name,PARAM_NAMESPACE);
    p.param<double>(ros::names::append(ns,"init_pose_x"), robot.initPose.x, 0.0);
    p.param<double>(ros::names::append(ns,"init_pose_y"), robot.initPose.y, 0.0);
    p.param<double>(ros::names::append(ns,"init_pose_yaw"), robot.initPose.theta, 0.0);
}

void CloudMapMerge::mapUpdate(const sensor_msgs::PointCloud2::ConstPtr& msg, CloudMapMerge::robotInfo& robot){
    //timestampが進んでいたらsensor_msgを更新してpclに変換する
    ROS_INFO_STREAM("suscribeThread << mapUpdate\n");
    std::lock_guard<std::mutex> lock(robot.mutex);
    if(robot.initialized && !(msg -> header.stamp > robot.rosMap.header.stamp)){
        return;
    }
    if(!robot.initialized){
        robot.pclMap = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        robot.initialized = true;
    }
    robot.rosMap = *msg;
    pcl::fromROSMsg(*msg, *robot.pclMap);
    robot.update = true;
}

void CloudMapMerge::mapMerging(void){
    ROS_INFO_STREAM("mergingThread << mapMerging\n");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Matrix2d rotation;
    //updateされているかどうかで処理を変える
    {
        boost::shared_lock<boost::shared_mutex> lock(robotListMutex);
        for(auto&& robot : robotList){
            if(!robot.initialized){
                continue;
            }
            std::lock_guard<std::mutex> lock(robot.mutex);
            if(robot.update){
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                rotation << cos(robot.initPose.theta) , -sin(robot.initPose.theta) , sin(robot.initPose.theta) , cos(robot.initPose.theta);
                for(const auto& point : robot.pclMap->points){
                    if(point.z < CEILING_HEIGHT && point.z > FLOOR_HEIGHT){
                        Eigen::Vector2d tempPoint(rotation * Eigen::Vector2d(point.x,point.y));
                        tempCloud -> points.emplace_back(ExpLib::pclXYZRGB(tempPoint.x() + robot.initPose.x, tempPoint.y() + robot.initPose.y,point.z,point.r,point.g,point.b));
                    }
                }
                robot.processedPclMap = std::move(tempCloud);
                robot.update = false;
            }
            *mergeCloud += *robot.processedPclMap;
        }
    }
    
    if(!mergeCloud -> points.empty()){
        ROS_DEBUG_STREAM("mergingThread << publish mergeCloud\n");
        mergeCloud -> width = mergeCloud -> points.size();
        mergeCloud -> height = 1;
        mergeCloud -> is_dense = false;

        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg (*mergeCloud, msg);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = MERGE_MAP_FRAME;
        pc2_.pub.publish(msg);
    }    
}

void CloudMapMerge::registrationLoop(void){
    ROS_INFO_STREAM("start registration loop\n");
    ros::Rate rate(RAGISTRATION_RATE);
    while(ros::ok()){
        robotRegistration();
        rate.sleep();
    }
}

void CloudMapMerge::mergingLoop(void){
    ROS_INFO_STREAM("start merging loop\n");
    ros::Rate rate(MERGING_RATE);
    while(ros::ok()){
        mapMerging();
        rate.sleep();
    }
}

void CloudMapMerge::multiThreadMainLoop(void){
    ROS_INFO_STREAM("start threads\n");
    ros::spinOnce();
    std::thread mergingThread([this]() { mergingLoop(); });
    std::thread registrationThread([this]() { registrationLoop(); });
    ros::spin();
    mergingThread.join();//スレッドの終了を待つ??
    registrationThread.join();
    ROS_INFO_STREAM("end main loop\n");
}