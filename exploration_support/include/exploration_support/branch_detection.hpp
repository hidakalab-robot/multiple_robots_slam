#ifndef BRANCH_DETECTION_HPP
#define BRANCH_DETECTION_HPP

#include <sensor_msgs/LaserScan.h>
#include <exploration_msgs/PointArray.h>
#include <exploration_libraly/struct.hpp>


class BranchDetection
{
private:
    double OBSTACLE_CHECK_ANGLE;
    double OBSTACLE_RANGE_MIX;

    ExpLib::subStructSimple scan_;
    ExpLib::pubStruct<exploration_msgs::PointArray> branch_;

    void scanCB(const sensor_msgs::LaserScan::ConstPtr& msg);
    void publishBranch(const std::vector<geometry_msgs::Point>& branches, const std::string& frameId);
public:
    BranchDetection();
};

BranchDetection::BranchDetection()
    :scan_("scan", 1, &BranchDetection::scanCB, this)
    ,branch_("branch",1){

    ros::NodeHandle p("~");
    p.param<double>("obstacle_check_angle", OBSTACLE_CHECK_ANGLE, 0.04);
    p.param<double>("obstacle_range_mix", OBSTACLE_RANGE_MIX,2.0);
    
}

void BranchDetection::scanCB(const sensor_msgs::LaserScan::ConstPtr& msg){
    // scan から branch を抽出してパブリッシュ

    for(int t=OBSTACLE_CHECK_ANGLE/msg->angle_increment,i=(msg->ranges.size()/2)-1-t,ie=(msg->ranges.size()/2)+t;i!=ie;++i){
		if(!std::isnan(msg->ranges[i]) && msg->ranges[i] < OBSTACLE_RANGE_MIX){
			ROS_ERROR_STREAM("It may be Close to Obstacles");
			return;
		}
    }

    std::vector<geometry_msgs::Point> branches;
    std::string frame;// poseから取ってくる?
    publishBranch(branches,frame);
}

void publishBranch(const std::vector<geometry_msgs::Point>& branches, const std::string& frameId){

}

#endif // BRANCH_DETECTION_HPP