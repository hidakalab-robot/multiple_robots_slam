#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Path.h>
#include <Eigen/Core>
#include <thread>


int main(int argc, char *argv[]){
    ros::init(argc, argv, "poselog_optimizer");

    ros::NodeHandle nh;
    message_filters::Subscriber<nav_msgs::Path> myPathSub(nh, "pose_log", 1);
    message_filters::Subscriber<nav_msgs::Path> slamPathSub(nh, "rtabmap/mapPath", 1);
    ros::Publisher pub = nh.advertise<nav_msgs::Path>("pose_log/optimized",1,true);
    double OPTIMIZE_TOLERANCE;
    double POSELOG_OPTIMIZED_PUBLISH_RATE;
    nh.param<double>("optimize_tolerance",OPTIMIZE_TOLERANCE,0.1);
    nh.param<double>("poselog_optimized_publish_rate", POSELOG_OPTIMIZED_PUBLISH_RATE, 10.0);

    nav_msgs::Path optimizedPath;

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Path, nav_msgs::Path> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), myPathSub, slamPathSub);
    sync.registerCallback(boost::bind(+[](const nav_msgs::PathConstPtr& mp,const nav_msgs::PathConstPtr& sp,const double& OPTIMIZE_TOLERANCE, nav_msgs::Path* optimizedPath)->void{
        // mpからspにないポーズを削除 <- rtabのポーズはタイムスタンプが書き換わってるため
        nav_msgs::Path tp = *mp;
        auto removeResult = std::remove_if(tp.poses.begin(),tp.poses.end(),[&](geometry_msgs::PoseStamped& p){
            for(const auto& s : sp->poses){
                if(Eigen::Vector2d(p.pose.position.x - s.pose.position.x, p.pose.position.y - s.pose.position.y).norm()<OPTIMIZE_TOLERANCE) return false;
            }
            return true;
        });
		tp.poses.erase(std::move(removeResult),tp.poses.end());
        *optimizedPath = tp;
    },_1,_2,OPTIMIZE_TOLERANCE,&optimizedPath));

    std::thread pubThread([&]{
        ros::Rate rate(POSELOG_OPTIMIZED_PUBLISH_RATE);
        while(ros::ok()){
            optimizedPath.header.stamp = ros::Time::now();
            pub.publish(optimizedPath);
            rate.sleep();
        }
    });


    ros::spin();
    pubThread.join();
    return 0;
}