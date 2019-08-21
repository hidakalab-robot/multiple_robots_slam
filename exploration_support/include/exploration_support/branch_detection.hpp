#ifndef BRANCH_DETECTION_HPP
#define BRANCH_DETECTION_HPP

#include <exploration_libraly/construct.hpp>
#include <exploration_libraly/struct.hpp>
#include <exploration_libraly/utility.hpp>
#include <exploration_msgs/PointArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

class BranchDetection
{
private:
    double OBSTACLE_CHECK_ANGLE;
    double OBSTACLE_RANGE_MIX;
    double SCAN_RANGE_THRESHOLD;
    double BRANCH_MAX_X;
	double BRANCH_DIFF_X_MIN;

    tf::TransformListener listener_;

    ExpLib::Struct::subStructSimple scan_;
    ExpLib::Struct::subStruct<geometry_msgs::PoseStamped> pose_;
    ExpLib::Struct::pubStruct<exploration_msgs::PointArray> branch_;

    void scanCB(const sensor_msgs::LaserScan::ConstPtr& msg);
    void publishBranch(const std::vector<geometry_msgs::Point>& branches, const std::string& frameId);
public:
    BranchDetection();
};

BranchDetection::BranchDetection()
    :scan_("scan", 1, &BranchDetection::scanCB, this)
    ,pose_("pose", 1)
    ,branch_("branch", 1){

    ros::NodeHandle p("~");
    p.param<double>("obstacle_check_angle", OBSTACLE_CHECK_ANGLE, 0.04);
    p.param<double>("obstacle_range_mix", OBSTACLE_RANGE_MIX,2.5);

    p.param<double>("scan_range_threshold", SCAN_RANGE_THRESHOLD, 6.0);
    p.param<double>("branch_max_x", BRANCH_MAX_X, 5.5);
	p.param<double>("branch_diff_x_min", BRANCH_DIFF_X_MIN, 1.0);
}

void BranchDetection::scanCB(const sensor_msgs::LaserScan::ConstPtr& msg){
    if(pose_.q.callOne(ros::WallDuration(1))){
        ROS_ERROR_STREAM("Can't read pose");
        publishBranch(std::vector<geometry_msgs::Point>(),pose_.data.header.frame_id);
        return;
    }

    static bool initialized = false;
    if(!initialized){
        listener_.waitForTransform(pose_.data.header.frame_id, msg->header.frame_id, ros::Time(), ros::Duration(1.0));
        initialized = true;
    }

    // センサと障害物の距離が近い時は検出を行わない
    for(int t=OBSTACLE_CHECK_ANGLE/msg->angle_increment,i=(msg->ranges.size()/2)-1-t,ie=(msg->ranges.size()/2)+t;i!=ie;++i){
		if(!std::isnan(msg->ranges[i]) && msg->ranges[i] < OBSTACLE_RANGE_MIX){
			// ROS_ERROR_STREAM("May be close to obstacles");
            publishBranch(std::vector<geometry_msgs::Point>(),pose_.data.header.frame_id);
            return;
		}
    }

    // nan のデータを取り除いて 距離と角度のデータに分ける
    ExpLib::Struct::scanStruct ss(msg->ranges.size(),msg->angle_max);

    for(int i=0,e=msg->ranges.size();i!=e;++i){
		if(!std::isnan(msg->ranges[i])){
			ss.ranges.push_back(msg->ranges[i]);
			ss.angles.push_back(msg->angle_min+(msg->angle_increment*i));
		}
    }

    if(ss.ranges.size() < 2){
		ROS_ERROR_STREAM("Scan data is insufficient");
        publishBranch(std::vector<geometry_msgs::Point>(),pose_.data.header.frame_id);
        return;
    }

    // 分岐検出部
    std::vector<geometry_msgs::Point> branches; // 検出した分岐領域を入れる

	//分岐のy座標の差がこの値の範囲内の場合のみ分岐として検出
	const float BRANCH_MIN_Y = BRANCH_DIFF_X_MIN*tan(ss.angleMax);//1.0 * tan(0.52) = 0.57
	const float BRANCH_MAX_Y = BRANCH_MAX_X*tan(ss.angleMax);//5.0 * tan(0.52) = 2.86 //この二つの差が正しいのでは // 6.0/tan(1.05)

    for(int i=0,e=ss.ranges.size()-1;i!=e;++i){
		//二つの角度の符号が違うときスキップ
		if(ss.angles[i] * ss.angles[i+1] < 0) continue;
		if(ss.ranges[i] < SCAN_RANGE_THRESHOLD && ss.ranges[i+1] < SCAN_RANGE_THRESHOLD){//距離が遠いのは信用できないのでだめ
			double x = ss.ranges[i] * cos(ss.angles[i]);
			double nextX = ss.ranges[i+1] * cos(ss.angles[i+1]);
			if(std::abs(nextX - x) >= BRANCH_DIFF_X_MIN){//x座標の差(分岐の幅)が一定以上じゃないと分岐と認めないフィルタ
				double y = ss.ranges[i] * sin(ss.angles[i]);
				double nextY = ss.ranges[i+1] * sin(ss.angles[i+1]);
				double diffY = std::abs(nextY - y);
				if(BRANCH_MIN_Y <= diffY && diffY <= BRANCH_MAX_Y){//分岐のy座標の差は一定の範囲に入っていないと分岐にしないフィルタ
                    // 検出した座標をpose座標系に変換して input
                    branches.emplace_back(ExpLib::Utility::coordinateConverter2d<geometry_msgs::Point>(listener_, pose_.data.header.frame_id, msg->header.frame_id, ExpLib::Construct::msgPoint((nextX + x)/2, (nextY + y)/2)));
				}
			}
		}
	}

    ROS_INFO_STREAM("Branch Found : " << branches.size());

    publishBranch(branches,pose_.data.header.frame_id);
}

void BranchDetection::publishBranch(const std::vector<geometry_msgs::Point>& branches, const std::string& frameId){
    exploration_msgs::PointArray msg;
    msg.points = branches;
    msg.header.frame_id = frameId;
    msg.header.stamp = ros::Time::now();
    branch_.pub.publish(msg);
    ROS_INFO_STREAM("Publish branch");
}

#endif // BRANCH_DETECTION_HPP