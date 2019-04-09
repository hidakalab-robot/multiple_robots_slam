#ifndef BRANCH_SEARCH_HPP
#define BRANCH_SEARCH_HPP

//分岐領域を検出するクラス
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <exploration_msgs/PointArray.h>
#include <std_msgs/Empty.h>
#include <exploration/common_lib.hpp>
#include <exploration/frontier_search.hpp>
#include <exploration_msgs/Frontier.h>

//分岐領域を検出
class BranchSearch
{
private:
	//パラメータ
	double SCAN_RANGE_THRESHOLD;
    double BRANCH_ANGLE;
    double CENTER_RANGE_MIN;
	double BRANCH_MAX_X;
	double BRANCH_DIFF_X_MIN;
	double DUPLICATE_TOLERANCE;
	bool DUPLICATE_CHECK;
	double LENGTH_THRESHOLD_Y;
	int LOG_NEWER_LIMIT;//if 30 -> ログの取得が1Hzの場合30秒前までのログで重複検出
	double FORWARD_DISTANCE;
	std::string MAP_FRAME_ID;

	CommonLib::subStruct<geometry_msgs::PoseArray> poseLog_;
	CommonLib::subStruct<sensor_msgs::LaserScan> scan_;
	CommonLib::subStruct<geometry_msgs::PoseStamped> pose_;

	CommonLib::pubStruct<geometry_msgs::PointStamped> goal_;
	CommonLib::pubStruct<exploration_msgs::PointArray> goalArray_;

	FrontierSearch fs;

	bool branchDetection(const CommonLib::scanStruct& ss,geometry_msgs::Point& goal,const geometry_msgs::Pose& pose);
    bool duplicateDetection(const geometry_msgs::Point& goal);
	void publishGoal(const geometry_msgs::Point& goal);
	void publishGoalArray(const std::vector<geometry_msgs::Point>& goals);

public:
    BranchSearch();
	bool getGoal(geometry_msgs::Point& goal);
};

BranchSearch::BranchSearch()
	:poseLog_("pose_log",1)
	,scan_("scan",1)
	,pose_("pose",1)
	,goal_("goal", 1, true)
	,goalArray_("goal_array", 1, true){

	ros::NodeHandle p("~");
	p.param<std::string>("map_frame_id", MAP_FRAME_ID, "map");
	p.param<double>("branch_angle", BRANCH_ANGLE, 0.04);
	p.param<double>("center_range_min", CENTER_RANGE_MIN, 1.0);
	p.param<double>("branch_max_x", BRANCH_MAX_X, 6.0);
	p.param<double>("branch_diff_x_min", BRANCH_DIFF_X_MIN, 1.0);
	p.param<double>("duplicate_tolerance", DUPLICATE_TOLERANCE, 1.5);
	p.param<bool>("duplicate_check", DUPLICATE_CHECK, true);
	p.param<double>("length_threshold_y", LENGTH_THRESHOLD_Y, 0.75);
	p.param<int>("log_newer_limit", LOG_NEWER_LIMIT, 30);
	p.param<double>("scan_range_threshold", SCAN_RANGE_THRESHOLD, 6.0);
	p.param<double>("forard_distance", FORWARD_DISTANCE, 2.0);
}

bool BranchSearch::getGoal(geometry_msgs::Point& goal){
	pose_.q.callOne(ros::WallDuration(1));
	scan_.q.callOne(ros::WallDuration(1));

	const int scanWidth = BRANCH_ANGLE / scan_.data.angle_increment;
    const int scanMin = (scan_.data.ranges.size()/2)-1 - scanWidth;
    const int scanMax = (scan_.data.ranges.size()/2) + scanWidth;

    for(int i = scanMin;i!=scanMax;++i){
		if(!std::isnan(scan_.data.ranges[i]) && scan_.data.ranges[i] < CENTER_RANGE_MIN){
			ROS_ERROR_STREAM("It may be Close to Obstacles\n");
			return false;
		}
    }

	CommonLib::scanStruct scanRect(scan_.data.ranges.size(),scan_.data.angle_max);

	for(int i=0,e=scan_.data.ranges.size();i!=e;++i){
		if(!std::isnan(scan_.data.ranges[i])){
			scanRect.ranges.push_back(scan_.data.ranges[i]);
			scanRect.angles.push_back(scan_.data.angle_min+(scan_.data.angle_increment*i));
		}
    }

	if(scanRect.ranges.size() < 2){
		ROS_ERROR_STREAM("scan_.data is Insufficient\n");
		return false;
    }

	if(branchDetection(scanRect,goal,pose_.data.pose)){
		publishGoal(goal);
		ROS_INFO_STREAM("Branch Found : (" << goal.x << "," << goal.y << ")\n");
		return true;
    }
	else{
		ROS_INFO_STREAM("Branch Do Not Found\n");
		return false;
	}
}

bool BranchSearch::branchDetection(const CommonLib::scanStruct& ss,geometry_msgs::Point& goal,const geometry_msgs::Pose& pose){
	ROS_DEBUG_STREAM("Searching Branch\n");

	//分岐のy座標の差がこの値の範囲内の場合のみ分岐として検出
	const float BRANCH_MIN_Y = BRANCH_DIFF_X_MIN*tan(ss.angleMax);//1.0 * tan(0.52) = 0.57
	const float BRANCH_MAX_Y = BRANCH_MAX_X*tan(ss.angleMax);//5.0 * tan(0.52) = 2.86 //この二つの差が正しいのでは // 6.0/tan(1.05)

	std::vector<geometry_msgs::Point> localList;
	localList.reserve(ss.ranges.size());

    //はじめのfor文では条件を満たした分岐領域を探す
    //２つ目のfor文で最も近い分岐を選ぶ
    //選んだ最も近い分岐に対して重複探査の確認を行いtrueが帰ってきたらその分岐を除いて再度最も近い分岐を探す
	//重複探査で全部弾かれた場合でもフロンティア領域との関係を見て

	for(int i=0,e=ss.ranges.size()-1;i!=e;++i){
		//二つの角度の符号が違うときスキップ
		if(ss.angles[i]*ss.angles[i+1]<0) continue;
		// if(scanX <= BRANCH_MAX_X && nextScanX <= BRANCH_MAX_X){//距離が遠い分岐は信用できないフィルタ
		if(ss.ranges[i] < SCAN_RANGE_THRESHOLD && ss.ranges[i+1] < SCAN_RANGE_THRESHOLD){//距離が遠いのは信用できないのでだめ
			double scanX = ss.ranges[i]*cos(ss.angles[i]);
			double nextScanX = ss.ranges[i+1]*cos(ss.angles[i+1]);
			if(std::abs(nextScanX - scanX) >= BRANCH_DIFF_X_MIN){//x座標の差(分岐の幅)が一定以上じゃないと分岐と認めないフィルタ
				double scanY = ss.ranges[i]*sin(ss.angles[i]);
				double nextScanY = ss.ranges[i+1]*sin(ss.angles[i+1]);
				double diffY = std::abs(nextScanY - scanY);
				if(BRANCH_MIN_Y <= diffY && diffY <= BRANCH_MAX_Y){//分岐のy座標の差は一定の範囲に入っていないと分岐にしないフィルタ
					double yLengthRight = Eigen::Vector2d(ss.ranges[i]*cos(ss.angles[i]) - ss.ranges[0]*cos(ss.angles[0]),ss.ranges[i]*sin(ss.angles[i]) - ss.ranges[0]*sin(ss.angles[0])).norm();
					double yLengthLeft = Eigen::Vector2d(ss.ranges[i+1]*cos(ss.angles[i+1]) - ss.ranges[e]*cos(ss.angles[e]),ss.ranges[i+1]*sin(ss.angles[i+1]) - ss.ranges[e]*sin(ss.angles[e])).norm();
					ROS_DEBUG_STREAM("yLengthRight : " << yLengthRight << ", yLengthLeft : " << yLengthLeft << "\n");
					//見つけた分岐から左右に一定以上センサデータが続いていないと分岐にしないフィルタ
					if(yLengthLeft > LENGTH_THRESHOLD_Y && yLengthRight > LENGTH_THRESHOLD_Y) localList.emplace_back(CommonLib::msgPoint((nextScanX + scanX)/2,(nextScanY + scanY)/2));
				}
			}
		}
	}
	
	if(localList.size()>0){
		ROS_DEBUG_STREAM("Branch Candidate Found : " << localList.size() << "\n");

		double yaw = CommonLib::qToYaw(pose.orientation);

		std::vector<geometry_msgs::Point> globalList;
		globalList.reserve(localList.size());
		
		for(const auto& l : localList) globalList.emplace_back(CommonLib::msgPoint(pose.position.x+(cos(yaw)*l.x)-(sin(yaw)*l.y),pose.position.y+(cos(yaw)*l.y)+(sin(yaw)*l.x)));

		publishGoalArray(globalList);

		std::vector<int> excludeList;
		excludeList.reserve(localList.size());

		for(int k=localList.size();k!=0;--k){
			float dist = DBL_MAX;
			int id;
			for(int j=0,e=localList.size();j!=e;++j){
				if(excludeList.size()>0 && std::any_of(excludeList.begin(), excludeList.end(), [j](int n){ return n == j; })) continue;
				float tempDist = std::abs(localList[j].x)+std::abs(localList[j].y);
				if(tempDist <= dist){
					dist = std::move(tempDist);
					goal = globalList[j];
					id = j;
				}
			}

			ROS_DEBUG_STREAM("Branch Candidate : (" << goal.x << "," << goal.y << ")\n");

			if(DUPLICATE_CHECK && duplicateDetection(goal)) excludeList.push_back(id);
			else return true;
		}

		//グローバルリストとフロンティア領域を比較して重複してても曲がるべきかを判断
		//残っているフロンティアに対してアクセスしやすい方向に進みたいので、角度の総和が小さい方が良い <- これ決定
		std::vector<exploration_msgs::Frontier> frontiers(fs.frontierDetection<std::vector<exploration_msgs::Frontier>>(false));
		if(frontiers.size()!=0){
			double min = DBL_MAX;
			for(int i=0,ei=globalList.size();i!=ei;++i){
				double sum = fs.sumFrontierAngle(globalList[i],Eigen::Vector2d(globalList[i].x-pose.position.x,globalList[i].y-pose.position.y).normalized(),frontiers);
				if(min > sum){
					min = std::move(sum);
					goal = globalList[i];
				}
			}
			//最後に直進方向のフロンティア面積と比較する //逆方向に行った時の奴も比較したほうが良いかも
			if(fs.sumFrontierAngle(pose,FORWARD_DISTANCE,frontiers) > min){
				ROS_DEBUG_STREAM("Branch : (" << goal.x << "," << goal.y << ")\n");
				ROS_DEBUG_STREAM("This Branch continues to a large frontier\n");
				return true;
			}
		}
    }

	return false;
}

bool BranchSearch::duplicateDetection(const geometry_msgs::Point& goal){
	poseLog_.q.callOne(ros::WallDuration(1));

	double xPlus = goal.x + DUPLICATE_TOLERANCE;
	double xMinus = goal.x - DUPLICATE_TOLERANCE;
	double yPlus = goal.y + DUPLICATE_TOLERANCE;
	double yMinus = goal.y - DUPLICATE_TOLERANCE;

	for(int i=0,e=poseLog_.data.poses.size()-LOG_NEWER_LIMIT;i!=e;++i){
		//過去のオドメトリが許容範囲の中に入っているか//
		if((xPlus >= poseLog_.data.poses[i].position.x) && (xMinus <= poseLog_.data.poses[i].position.x)){
			if((yPlus >= poseLog_.data.poses[i].position.y) && (yMinus <= poseLog_.data.poses[i].position.y)){
				ROS_DEBUG_STREAM("This Branch is Duplicated\n");
				return true;
			}
		}
	}
	ROS_DEBUG_STREAM("This Branch is Not Duplicated\n");
	return false;
}

void BranchSearch::publishGoal(const geometry_msgs::Point& goal){
	geometry_msgs::PointStamped msg;
	msg.point = goal;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = MAP_FRAME_ID;
	goal_.pub.publish(msg);
	ROS_INFO_STREAM("Publish Goal\n");
}

void BranchSearch::publishGoalArray(const std::vector<geometry_msgs::Point>& goals){
	exploration_msgs::PointArray msg;
	msg.points = goals;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = MAP_FRAME_ID;
	goalArray_.pub.publish(msg);
	ROS_INFO_STREAM("Publish GoalList\n");
}

#endif //BRANCH_SEARCH_HPP