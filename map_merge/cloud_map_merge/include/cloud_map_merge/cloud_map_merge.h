#ifndef CLOUD_MAP_MERGE_H
#define CLOUD_MAP_MERGE_H

#include <boost/thread.hpp>
#include <exploration_libraly/struct.h>
#include <exploration_libraly/construct.h>
#include <forward_list>
#include <geometry_msgs/Pose2D.h>
#include <iterator>
#include <mutex>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>

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

    boost::shared_mutex robotListMutex_;
    ros::NodeHandle s_;
    ros::NodeHandle p_;

    ExpLib::Struct::pubStruct<sensor_msgs::PointCloud2> pc2_;

    std::forward_list<robotInfo> robotList_;

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

#endif // CLOUD_MAP_MERGE_H