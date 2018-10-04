#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include <map_merge/RobotPose.h>
#include <map_merge/AllRobotData.h>
#include <pcl_ros/point_cloud.h>

class CloudMapMerge
{
private:
    ros::NodeHandle cmm;

    ros::NodeHandle s;

    ros::NodeHandle p;

    ros::Subscriber sub;
    ros::Publisher pub;
    map_merge::AllRobotData robotData;
    sensor_msgs::PointCloud2 pubCloud;
    std::string subTopic;
    std::string mergeMapFrame;
    //std::string pubTopic;

    bool input;

    void callback(const map_merge::AllRobotData::ConstPtr& msg);

public:
    ros::CallbackQueue queue;
    CloudMapMerge();
    ~CloudMapMerge(){};

    void translation(void);
    void checkOverlap(void);
    void merge(void);
    bool isInput(void);
    void resetFlag(void);
    void publish(void);

};

CloudMapMerge::CloudMapMerge()
:cmm("~")
{
    input = false;
    s.setCallbackQueue(&queue);
    cmm.getParam("sub_topic", subTopic);
    cmm.getParam("merge_map_frame", mergeMapFrame);
    sub = s.subscribe(subTopic,1,&CloudMapMerge::callback,this);
    pub = p.advertise<sensor_msgs::PointCloud2>("cloud_map_merge/merge_map", 1);
}

void CloudMapMerge::callback(const map_merge::AllRobotData::ConstPtr& msg)
{
    robotData = *msg;
    input = true;
    std::cout << "input robot data" << std::endl;
}

bool CloudMapMerge::isInput(void)
{
  return input;
}

void CloudMapMerge::resetFlag(void)
{
  input = false;
}

void CloudMapMerge::checkOverlap(void)
{
    /*ここでマップの重なりを検出してposesの書き換えを行う*/
    /*どこが重なってるか分かったら、その部分だけ抽出したクラウドを作る*/
}

void CloudMapMerge::merge(void)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i=0;i<robotData.poses.size();i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg (robotData.maps[i], *transCloud);

        Eigen::Matrix2d rotation;
        rotation << cos(robotData.poses[i].yaw) , -sin(robotData.poses[i].yaw) , sin(robotData.poses[i].yaw) , cos(robotData.poses[i].yaw);

        Eigen::Vector2d point;
        Eigen::Vector2d rotatePoint;

        for(int j=0;j<transCloud -> points.size();j++)
        {
            point << transCloud -> points[j].x , transCloud -> points[j].y;
            rotatePoint = rotation * point;
            transCloud -> points[j].x = rotatePoint[0] + robotData.poses[i].x;
            transCloud -> points[j].y = rotatePoint[1] + robotData.poses[i].y;
        }

        *mergedCloud += *transCloud;

    }

    mergedCloud -> width = mergedCloud -> points.size();
    mergedCloud -> height = 1;
    mergedCloud -> is_dense = false;
    pcl::toROSMsg (*mergedCloud, pubCloud);


}

void CloudMapMerge::publish(void)
{
    pubCloud.header.stamp = ros::Time::now();
    pubCloud.header.frame_id = mergeMapFrame;
    pub.publish(pubCloud);
}