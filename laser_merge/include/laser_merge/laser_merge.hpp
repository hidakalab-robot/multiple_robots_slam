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
    struct listenerStruct{
        tf::TransformListener* listener;
        bool initialized;
        listenerStruct():initialized(false){};
    };
    std::string MERGE_LASER_FRAME;
    int LASER_NUBER;

    laser_geometry::LaserProjection lp;

    // std::vector<tf::TransformListener> scanListener;

    // tf::TransformListener scan1Listener;
    // tf::TransformListener scan2Listener;
    std::vector<listenerStruct> ls;

    CommonLib::pubStruct<sensor_msgs::PointCloud2> pc2_;

public:
    LaserMerge();
    void callback(const sensor_msgs::LaserScanConstPtr& scan1,const sensor_msgs::LaserScanConstPtr& scan2);
};

LaserMerge::LaserMerge():pc2_("cloud_out",1){
    ros::NodeHandle p("~");
    
    p.param<int>("laser_number",LASER_NUBER,2);
    p.param<std::string>("merge_laser_frame",MERGE_LASER_FRAME,"merge_laser_link");

    ls.resize(LASER_NUBER);
}

void LaserMerge::callback(const sensor_msgs::LaserScanConstPtr& scan1,const sensor_msgs::LaserScanConstPtr& scan2){

    ROS_INFO_STREAM("callback");

    std::vector<sensor_msgs::LaserScan> scan(LASER_NUBER);
    // ここだけ直接代入になってる
    scan[0] = *scan1;
    scan[1] = *scan2;

    std::vector<sensor_msgs::PointCloud2> cloud(LASER_NUBER);
    
    for(int i=0,ie=ls.size();i!=ie;++i){
        if(!ls[i].initialized){
            ls[i].listener->waitForTransform(MERGE_LASER_FRAME, scan[i].header.frame_id, ros::Time(), ros::Duration(1.0));
            ls[i].initialized = true;
        }
        tf::StampedTransform transform;
        ls[i].listener->lookupTransform(MERGE_LASER_FRAME, scan[i].header.frame_id, ros::Time(0), transform);
        lp.projectLaser(scan[i],cloud[i]);

        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud[i],*tempCloud);

        double transX = transform.getOrigin().getX();
        double transY = transform.getOrigin().getY();
        double transYaw = CommonLib::qToYaw(transform.getRotation());

        Eigen::Matrix2d rotation;
        rotation << cos(transYaw) , -sin(transYaw) , sin(transYaw) , cos(transYaw);

        for(auto&& p : tempCloud->points) {
            Eigen::Vector2d tempPoint(rotation * Eigen::Vector2d(p.x - transX, p.y - transY));
            p.x = tempPoint.x();
            p.y = tempPoint.y();
            p.z = 0;
        }
        pcl::toROSMsg(*tempCloud,cloud[i]);
    }

    sensor_msgs::PointCloud2 mergeCloud;
    pcl::concatenatePointCloud(cloud[0],cloud[1],mergeCloud);

    mergeCloud.header.frame_id = MERGE_LASER_FRAME;
    mergeCloud.header.stamp = ros::Time::now();

    pc2_.pub.publish(mergeCloud);


        //ここで合成の処理をする
    // static bool scan1TfInitialized = false;
    // static bool scan2TfInitialized = false;

    // if(!scan1TfInitialized){
    //     scan1Listener.waitForTransform(MERGE_LASER_FRAME, scan1->header.frame_id, ros::Time(), ros::Duration(1.0));
    //     scan1TfInitialized = true;
    // }
    // if(!scan2TfInitialized){
    //     scan2Listener.waitForTransform(MERGE_LASER_FRAME, scan2->header.frame_id, ros::Time(), ros::Duration(1.0));
    //     scan2TfInitialized = true;
    // }

    // tf::StampedTransform scan1Transform, scan2Transform;
    // scan1Listener.lookupTransform(MERGE_LASER_FRAME, scan1->header.frame_id, ros::Time(0), scan1Transform);
    // scan2Listener.lookupTransform(MERGE_LASER_FRAME, scan2->header.frame_id, ros::Time(0), scan2Transform);

    // ROS_INFO_STREAM("callback");

    //scan2 urg

    // sensor_msgs::PointCloud2 cloud1, cloud2, mergeCloud;

    // sensor_msgs::PointCloud2 mergeCloud;

    // lp.projectLaser(*scan1,cloud1);
    // lp.projectLaser(*scan2,cloud2);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud1(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud2(new pcl::PointCloud<pcl::PointXYZ>);

    // pcl::fromROSMsg(cloud1,*tempCloud1);
    // pcl::fromROSMsg(cloud2,*tempCloud2);

    // 両方高さを0
    // double trans1Yaw = CommonLib::qToYaw(scan1Transform.getRotation());
    // double trans1X = scan1Transform.getOrigin().getX();
    // double trans1Y = scan1Transform.getOrigin().getY();

    // double trans2Yaw = CommonLib::qToYaw(scan2Transform.getRotation());
    // double trans2X = scan2Transform.getOrigin().getX();
    // double trans2Y = scan2Transform.getOrigin().getY();

    // Eigen::Matrix2d rotation1;
    // rotation1 << cos(trans1Yaw) , -sin(trans1Yaw) , sin(trans1Yaw) , cos(trans1Yaw);

    // Eigen::Matrix2d rotation2;
    // rotation2 << cos(trans2Yaw) , -sin(trans2Yaw) , sin(trans2Yaw) , cos(trans2Yaw);

    // for(auto&& p : tempCloud1->points) {
    //     Eigen::Vector2d tempPoint(rotation1 * Eigen::Vector2d(p.x - trans1X, p.y - trans1Y));
    //     p.x = tempPoint.x();
    //     p.y = tempPoint.y();
    //     p.z = 0;
    // }
    //urdfでkinectの高さもしくはレーザーの高さ調節
    //マージしたスキャンデータをslamにいれてみる
    //mapping launch を分ける, urdf も分ける

    // Eigen::Matrix2d rotation;
    // rotation << cos(M_PI) , -sin(M_PI) , sin(M_PI) , cos(M_PI);

    // for(auto&& p : tempCloud2->points) {
    //     Eigen::Vector2d tempPoint(rotation2 * Eigen::Vector2d(p.x - trans2X, p.y - trans2Y));
    //     p.x = tempPoint.x();
    //     p.y = tempPoint.y();
    //     p.z = 0;
    // }

    // pcl::PointCloud<pcl::PointXYZ>::Ptr pclMergeCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // *pclMergeCloud = *tempCloud1 + *tempCloud2;

    // pcl::toROSMsg(*tempCloud1,cloud1);
    // pcl::toROSMsg(*tempCloud2,cloud2);

    // pcl::toROSMsg(*pclMergeCloud,mergeCloud);


    // 合成はこっちの方がいい感じにしてくれる気がする
    // pcl::concatenatePointCloud(cloud1,cloud2,mergeCloud);

    // for(auto&& p : tempCloud2->points) {
    //     Eigen::Vector2d tempPoint(rotation * Eigen::Vector2d(p.x,p.y));
    //     p.x = tempPoint.x();
    //     p.y = tempPoint.y();
    //     p.z = 0;
    // }

    // mergeCloud.header.frame_id = MERGE_LASER_FRAME;
    // mergeCloud.header.stamp = ros::Time::now();

    // pc2_.pub.publish(mergeCloud);

}