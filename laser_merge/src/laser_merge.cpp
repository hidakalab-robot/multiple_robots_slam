#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <exploration_libraly/struct.hpp>
#include <exploration_libraly/convert.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <exploration_libraly/utility.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "laser_merge");

    ros::NodeHandle nh("~");
    std::string SCAN_TOPIC_ONE;
    std::string SCAN_TOPIC_TWO;
    std::string MERGE_LASER_FRAME;

    nh.param<std::string>("scan_topic_one",SCAN_TOPIC_ONE,"scan1");
    nh.param<std::string>("scan_topic_two",SCAN_TOPIC_TWO,"scan2");
    nh.param<std::string>("merge_laser_frame",MERGE_LASER_FRAME,"merge_laser_link");
    
    message_filters::Subscriber<sensor_msgs::LaserScan> scan1Sub(nh,SCAN_TOPIC_ONE,1);
    message_filters::Subscriber<sensor_msgs::LaserScan> scan2Sub(nh,SCAN_TOPIC_TWO,1);
    using MySyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan>;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), scan1Sub, scan2Sub);

    sync.registerCallback(boost::bind(+[](const sensor_msgs::LaserScanConstPtr& scan1,const sensor_msgs::LaserScanConstPtr& scan2, const std::string& mlf)->void{
        static int LASER_NUMBER = 2;
        static laser_geometry::LaserProjection lp_;
        static ExpLib::pubStruct<sensor_msgs::PointCloud2> pc2("cloud_out",1);
        static std::vector<tf::TransformListener> listener(LASER_NUMBER);
        static std::vector<bool> initialized(LASER_NUMBER);

        std::vector<sensor_msgs::LaserScan> scan(LASER_NUMBER);
        scan[0] = *scan1;
        scan[1] = *scan2;

        std::vector<sensor_msgs::PointCloud2> cloud(LASER_NUMBER);
        for(int i=0,ie=cloud.size();i!=ie;++i){
            if(!initialized[i]){
                listener[i].waitForTransform(mlf, scan[i].header.frame_id, ros::Time(), ros::Duration(1.0));
                initialized[i] = true;
            }
            lp_.projectLaser(scan[i],cloud[i]);
            pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(cloud[i],*tempCloud);
            for(auto&& p : tempCloud->points) ExpLib::coordinateConverter2d<void>(listener[i],mlf,scan[i].header.frame_id,p);
            pcl::toROSMsg(*tempCloud,cloud[i]);
        }

        sensor_msgs::PointCloud2 mergeCloud;
        pcl::concatenatePointCloud(cloud[0],cloud[1],mergeCloud);
        mergeCloud.header.frame_id = mlf;
        mergeCloud.header.stamp = ros::Time::now();
        pc2.pub.publish(mergeCloud);
    },_1,_2,MERGE_LASER_FRAME));
    
    ros::spin();
    return 0;
}