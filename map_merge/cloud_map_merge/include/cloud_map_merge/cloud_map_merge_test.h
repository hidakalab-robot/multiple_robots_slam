#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose2D.h>
#include <pcl_ros/point_cloud.h>
#include <thread>
#include <mutex>
#include <boost/thread.hpp>
#include <forward_list>
#include <iterator>

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
        bool initialized;
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

    double RAGISTRATION_RATE;
    double MERGING_RATE;

    ros::Publisher mapPub;

    void robotRegistration(void);//ロボットの情報を登録
    bool isMapTopic(ros::master::TopicInfo& topic);
    std::string robotNameFromTopicName(std::string& topicName);
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
    param.param<double>("ragistration_rate", RAGISTRATION_RATE, 0.5);
    param.param<double>("merging_rate", MERGING_RATE, 1.0);
    mapPub = pub.advertise<sensor_msgs::PointCloud2>("merge_map",1,true);
}

void CloudMapMerge::robotRegistration(void){
    //masterに登録されているtopicを読み込み
    ros::master::V_TopicInfo topicList;
    ros::master::getTopics(topicList);
    ROS_INFO_STREAM("registrationThread << robotRegistration\n");
    //topicListの中からmapのトピックのみを抽出
    //for(int i=0;i<topicList.size();++i){
    for(auto& topic : topicList){
        //maptopicであるか確認
        //ROS_DEBUG_STREAM("registrationThread << topic name : " << topic.name << "\n");
        if(!isMapTopic(topic)){
            //ROS_DEBUG_STREAM("registrationThread << continue\n");
            continue;
        }
        else{
             //ROS_DEBUG_STREAM("registrationThread << this is map topic\n");
        }
        std::string robotName = robotNameFromTopicName(topic.name);
        //ROS_DEBUG_STREAM("registrationThread << check registration\n");
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
                //ROS_DEBUG_STREAM("registrationThread << this robot is registrated\n");
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
                //robot.mapSub = sub.subscribe<sensor_msgs::PointCloud2>(robot.name+MAP_TOPIC, 1,boost::bind(&CloudMapMerge::mapUpdate,this,_1,robot));
                robot.mapSub = sub.subscribe<sensor_msgs::PointCloud2>(robot.name+MAP_TOPIC, 1, [this, &robot](const sensor_msgs::PointCloud2::ConstPtr& msg) {mapUpdate(msg, robot);});
                robot.initialized = false;
            }
        }
    }
}

bool CloudMapMerge::isMapTopic(ros::master::TopicInfo& topic){
    //ROS_DEBUG_STREAM("registrationThread << isMapTopic\n");
    bool isMap = topic.name == ros::names::append(robotNameFromTopicName(topic.name),MAP_TOPIC);
    //ROS_DEBUG_STREAM("registrationThread << topic.name : " << topic.name << "\n");
    //ROS_DEBUG_STREAM("registrationThread << parent.name : " << ros::names::parentNamespace(ros::names::parentNamespace(topic.name)) << "\n");
    //ROS_DEBUG_STREAM("registrationThread << append.name : " << ros::names::append(ros::names::parentNamespace(ros::names::parentNamespace(topic.name)),MAP_TOPIC) << "\n");
    //ROS_DEBUG_STREAM("registrationThread << isMap : " << isMap << "\n");
    bool isPointCloud = topic.datatype == "sensor_msgs/PointCloud2";
    //ROS_DEBUG_STREAM("registrationThread << topic.datatype : " << topic.datatype << "\n");
    //ROS_DEBUG_STREAM("registrationThread << isPointCloud : " << isPointCloud << "\n");
    return isMap && isPointCloud;
}

std::string CloudMapMerge::robotNameFromTopicName(std::string& topicName){
    //ros::names::parentNamespace(std::string);はトピック名からネームスペースの文字列を返す
    //例
    ///robot1/map -> /robot1
    ///robot1/abc/map -> /robot1/abc
    return ros::names::parentNamespace(ros::names::parentNamespace(topicName));
}

void CloudMapMerge::initPoseLoad(CloudMapMerge::robotInfo& robot){
    //ROS_DEBUG_STREAM("registrationThread << initPoseLoad\n");
    std::string ns = ros::names::append(robot.name,PARAM_NAMESPACE);
    param.param<double>(ros::names::append(ns,"init_pose_x"), robot.initPose.x, 0.0);
    param.param<double>(ros::names::append(ns,"init_pose_y"), robot.initPose.y, 0.0);
    param.param<double>(ros::names::append(ns,"init_pose_yaw"), robot.initPose.theta, 0.0);
}

void CloudMapMerge::mapUpdate(const sensor_msgs::PointCloud2::ConstPtr& msg, CloudMapMerge::robotInfo& robot){
    //timestampが進んでいたらsensor_msgを更新してpclに変換する
    ROS_INFO_STREAM("suscribeThread << mapUpdate\n");
    std::lock_guard<std::mutex> lock(robot.mutex);
    if(robot.initialized && robot.rosMap.header.stamp > msg -> header.stamp){
        //ROS_DEBUG_STREAM("suscribeThread << return\n");
        return;
    }
    //ROS_DEBUG_STREAM("suscribeThread << input new data\n");
    if(!robot.initialized){
        robot.pclMap = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        robot.initialized = true;
    }
    robot.rosMap = *msg;
    pcl::fromROSMsg(*msg, *robot.pclMap);
}

void CloudMapMerge::mapMerging(void){
    ROS_INFO_STREAM("mergingThread << mapMerging\n");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Matrix2d rotation;
    Eigen::Vector2d tempPoint;
    Eigen::Vector2d rotatePoint;
    {
        boost::shared_lock<boost::shared_mutex> lock(robotListMutex);
        //for(int i=0;i<robotList.size();++i){
        //ROS_INFO_STREAM("mergingThread << iterate list size : " << std::distance(robotList.begin(), robotList.end()) << "\n");
        for(auto& robot : robotList){
            if(!robot.initialized){
                continue;
            }
            std::lock_guard<std::mutex> lock(robot.mutex);
            rotation << cos(robot.initPose.theta) , -sin(robot.initPose.theta) , sin(robot.initPose.theta) , cos(robot.initPose.theta);
            //ROS_INFO_STREAM("mergingThread << iterate points size " << robot.pclMap->points.size() << "\n");
            for(auto& point : robot.pclMap->points){
                if(point.z < CEILING_HEIGHT && point.z > FLOOR_HEIGHT){
                    tempPoint << point.x , point.y;
                    rotatePoint = rotation * tempPoint;
                    mergeCloud -> points.emplace_back(pcl::PointXYZRGB());
                    pcl::PointXYZRGB& assignedPoint = mergeCloud -> points.back();
                    //insertPoint = point;
                    assignedPoint.x = rotatePoint.x() + robot.initPose.x;
                    assignedPoint.y = rotatePoint.y() + robot.initPose.y;
                    assignedPoint.z = point.z;
                    assignedPoint.r = point.r;
                    assignedPoint.g = point.g;
                    assignedPoint.b = point.b;
                }
            }
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
        mapPub.publish(msg);
    }
    // else{
    //     ROS_INFO_STREAM("mergingThread << mergeCloud is empty\n");
    // }
    
}

void CloudMapMerge::registrationLoop(void){
    ROS_INFO_STREAM("start registration loop\n");
    ros::Rate rate(RAGISTRATION_RATE);
    while(ros::ok()){
        //ROS_INFO_STREAM("registrationThread << registration loop\n");
        robotRegistration();
        rate.sleep();
    }
}

void CloudMapMerge::mergingLoop(void){
    ROS_INFO_STREAM("start merging loop\n");
    ros::Rate rate(MERGING_RATE);
    while(ros::ok()){
        //ROS_INFO_STREAM("mergingThread << merging loop\n");
        mapMerging();
        rate.sleep();
    }
}

void CloudMapMerge::multiThreadMainLoop(void){
    ROS_INFO_STREAM("start threads\n");
    ros::spinOnce();
    std::thread mergingThread([this]() { mergingLoop(); });
    std::thread registrationThread([this]() { registrationLoop(); });
    //std::thread mergingThread(&CloudMapMerge::mergingLoop,this);
    //std::thread registrationThread(&CloudMapMerge::registrationLoop,this);
    ros::spin();
    mergingThread.join();//スレッドの終了を待つ??
    registrationThread.join();
    ROS_INFO_STREAM("end main loop\n");
}