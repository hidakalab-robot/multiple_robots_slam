// ここではレーザーデータをポイントクラウドに変換して合成する　そしてそれをトピックに投げると変換してくれる
#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <exploration/common_lib.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Geometry>

class LaserMerge{
private:
    // std::string SCAN_TOPIC_ONE;
    // std::string SCAN_TOPIC_TWO;

    std::string MERGE_LASER_FRAME;

    // ros::NodeHandle nh;

    // message_filters::Subscriber<sensor_msgs::LaserScan> scan1Sub;
    // message_filters::Subscriber<sensor_msgs::LaserScan> scan2Sub;
    // message_filters::TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::LaserScan> sync;

    laser_geometry::LaserProjection lp;

    tf::TransformListener scan1Listener;
    tf::TransformListener scan2Listener;

    CommonLib::pubStruct<sensor_msgs::PointCloud2> pc2_;

    

public:
    LaserMerge();
    void callback(const sensor_msgs::LaserScanConstPtr& scan1,const sensor_msgs::LaserScanConstPtr& scan2);
};

LaserMerge::LaserMerge():pc2_("cloud_out",1){// 引数がパラムのやつなので無理
    ros::NodeHandle p("~");
    p.param<std::string>("merge_laser_frame",MERGE_LASER_FRAME,"merge_laser");
}

void LaserMerge::callback(const sensor_msgs::LaserScanConstPtr& scan1,const sensor_msgs::LaserScanConstPtr& scan2){
    //ここで合成の処理をする
    static bool scan1TfInitialized = false;
    static bool scan2TfInitialized = false;

    if(!scan1TfInitialized){
        scan1Listener.waitForTransform(scan1->header.frame_id, MERGE_LASER_FRAME, ros::Time(), ros::Duration(1.0));
        scan1TfInitialized = true;
    }

    if(!scan2TfInitialized){
        scan2Listener.waitForTransform(scan2->header.frame_id, MERGE_LASER_FRAME, ros::Time(), ros::Duration(1.0));
        scan2TfInitialized = true;
    }
    
    tf::StampedTransform scan1Transform;
    scan1Listener.lookupTransform(scan1->header.frame_id, MERGE_LASER_FRAME, ros::Time(0), scan1Transform);

    tf::StampedTransform scan2Transform;
    scan2Listener.lookupTransform(scan2->header.frame_id, MERGE_LASER_FRAME, ros::Time(0), scan2Transform);

    ROS_INFO_STREAM("callback");

    //scan2 urg

    sensor_msgs::PointCloud2 cloud1;
    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::PointCloud2 mergeCloud;

    lp.projectLaser(*scan1,cloud1);
    lp.projectLaser(*scan2,cloud2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(cloud1,*tempCloud1);
    pcl::fromROSMsg(cloud2,*tempCloud2);

    // 両方高さを0
    for(auto&& p : tempCloud1->points) {
        p.z = 0;
    }
    //urdfでkinectの高さもしくはレーザーの高さ調節
    //マージしたスキャンデータをslamにいれてみる
    //mapping launch を分ける, urdf も分ける

    

    Eigen::Matrix2d rotation;
    rotation << cos(M_PI) , -sin(M_PI) , sin(M_PI) , cos(M_PI);

    for(auto&& p : tempCloud2->points) {
        Eigen::Vector2d tempPoint(rotation * Eigen::Vector2d(p.x,p.y));
        p.x = tempPoint.x();
        p.y = tempPoint.y();
        p.z = 0;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclMergeCloud(new pcl::PointCloud<pcl::PointXYZ>);

    *pclMergeCloud = *tempCloud1 + *tempCloud2;

    // pcl::toROSMsg(*tempCloud1,cloud1);
    // pcl::toROSMsg(*tempCloud2,cloud2);

    pcl::toROSMsg(*pclMergeCloud,mergeCloud);


    // 合成はこっちの方がいい感じにしてくれる気がする
    // pcl::concatenatePointCloud(cloud1,cloud2,mergeCloud);

    // for(auto&& p : tempCloud2->points) {
    //     Eigen::Vector2d tempPoint(rotation * Eigen::Vector2d(p.x,p.y));
    //     p.x = tempPoint.x();
    //     p.y = tempPoint.y();
    //     p.z = 0;
    // }

    mergeCloud.header.frame_id = MERGE_LASER_FRAME;
    mergeCloud.header.stamp = ros::Time::now();

    pc2_.pub.publish(mergeCloud);

}

// int main(int argc, char** argv){
//     ros::init(argc, argv, "laser_merge");

//     ros::NodeHandle nh("~");

//     std::string SCAN_TOPIC_ONE;
//     std::string SCAN_TOPIC_TWO;

//     nh.param<std::string>("scan_topic_one",SCAN_TOPIC_ONE,"scan1");
//     nh.param<std::string>("scan_topic_two",SCAN_TOPIC_TWO,"scan2");
    

//     message_filters::Subscriber<sensor_msgs::LaserScan> scan1Sub(nh,SCAN_TOPIC_ONE,1);
//     message_filters::Subscriber<sensor_msgs::LaserScan> scan2Sub(nh,SCAN_TOPIC_TWO,1);
//     message_filters::TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::LaserScan> sync(scan1Sub,scan2Sub,10);

//     LaserMerge lm;

//     sync.registerCallback(boost::bind(&LaserMerge::callback,&lm,_1,_2));

    
//     ros::spin();
//     return 0;
// }