#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <exploration_libraly/common_lib.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Geometry>

class LaserMerge{
private:
    std::string MERGE_LASER_FRAME;
    int LASER_NUMBER;

    laser_geometry::LaserProjection lp;
    CommonLib::pubStruct<sensor_msgs::PointCloud2> pc2_;

public:
    LaserMerge();
    void callback(const sensor_msgs::LaserScanConstPtr& scan1,const sensor_msgs::LaserScanConstPtr& scan2);
};

LaserMerge::LaserMerge():pc2_("cloud_out",1){
    ros::NodeHandle p("~");
    p.param<int>("laser_number",LASER_NUMBER,2);
    p.param<std::string>("merge_laser_frame",MERGE_LASER_FRAME,"merge_laser_link");
}

void LaserMerge::callback(const sensor_msgs::LaserScanConstPtr& scan1,const sensor_msgs::LaserScanConstPtr& scan2){
    std::vector<sensor_msgs::LaserScan> scan(LASER_NUMBER);
    // ここだけ直接代入になってる
    scan[0] = *scan1;
    scan[1] = *scan2;

    std::vector<sensor_msgs::PointCloud2> cloud(LASER_NUMBER);
    static std::vector<tf::TransformListener> listener(LASER_NUMBER);
    static std::vector<bool> initialized(LASER_NUMBER);
    
    for(int i=0,ie=scan.size();i!=ie;++i){
        if(!initialized[i]){
            listener[i].waitForTransform(MERGE_LASER_FRAME, scan[i].header.frame_id, ros::Time(), ros::Duration(1.0));
            initialized[i] = true;
        }
        tf::StampedTransform transform;
        listener[i].lookupTransform(MERGE_LASER_FRAME, scan[i].header.frame_id, ros::Time(0), transform);
        lp.projectLaser(scan[i],cloud[i]);

        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud[i],*tempCloud);

        double transX = transform.getOrigin().getX();
        double transY = transform.getOrigin().getY();
        double transYaw = CommonLib::qToYaw(transform.getRotation());

        ROS_INFO_STREAM("frame: " << scan[i].header.frame_id << ", transX: " << transX << ", transY: " << transY);

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
}