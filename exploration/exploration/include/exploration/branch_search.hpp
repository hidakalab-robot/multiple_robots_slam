#ifndef BRANCH_SEARCH_HPP
#define BRANCH_SEARCH_HPP

//分岐領域を検出するクラス
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <exploration_msgs/PoseStampedArray.h>
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
	enum Duplication{
		NOT_DUPLECATION,
		OLDER,
		NEWER
	};
	struct listStruct{
		geometry_msgs::Point point;
		Duplication duplication;
		listStruct(const geometry_msgs::Point& p):point(p){};
	};
	//パラメータ
	double SCAN_RANGE_THRESHOLD;
    double BRANCH_ANGLE;
    double CENTER_RANGE_MIN;
	double BRANCH_MAX_X;
	double BRANCH_DIFF_X_MIN;
	double BRANCH_TOLERANCE;
	double DUPLICATE_TOLERANCE;
	bool DUPLICATE_CHECK;
	double LENGTH_THRESHOLD_Y;
	double LOG_NEWER_LIMIT;//if 30 -> 30秒前までのログで重複検出
	std::string MAP_FRAME_ID;
	double THROUGH_TOLERANCE;
	bool ACTIVE_HIBRID;
	double NEWER_DUPLICATION_THRESHOLD;//最近通った場所の重複とみなす時間の上限,時間の仕様はLOG_NEWER_LIMITと同じ

	CommonLib::subStruct<exploration_msgs::PoseStampedArray> poseLog_;
	CommonLib::subStruct<sensor_msgs::LaserScan> scan_;
	CommonLib::subStruct<geometry_msgs::PoseStamped> pose_;

	CommonLib::pubStruct<geometry_msgs::PointStamped> goal_;
	CommonLib::pubStruct<exploration_msgs::PointArray> goalArray_;

	FrontierSearch fs;
	std::vector<geometry_msgs::Point> throughBranch;//一度重複探査を無視して行った座標（二回目は行けない）

	bool branchDetection(const CommonLib::scanStruct& ss,geometry_msgs::Point& goal,const geometry_msgs::Pose& pose);
    Duplication duplicateDetection(const geometry_msgs::Point& goal);
	void publishGoal(const geometry_msgs::Point& goal);
	void publishGoalArray(const std::vector<geometry_msgs::Point>& goals);
	std::vector<geometry_msgs::Point> listStructToPoint(const std::vector<listStruct>& list);

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
	p.param<double>("center_range_min", CENTER_RANGE_MIN,2.0);
	p.param<double>("branch_max_x", BRANCH_MAX_X, 5.5);
	p.param<double>("branch_diff_x_min", BRANCH_DIFF_X_MIN, 1.0);
	p.param<double>("duplicate_tolerance", DUPLICATE_TOLERANCE, 1.5);
	p.param<bool>("duplicate_check", DUPLICATE_CHECK, true);
	p.param<double>("length_threshold_y", LENGTH_THRESHOLD_Y, 0.75);
	p.param<double>("log_newer_limit", LOG_NEWER_LIMIT, 10);
	p.param<double>("scan_range_threshold", SCAN_RANGE_THRESHOLD, 6.0);
	p.param<double>("through_tolerance", THROUGH_TOLERANCE, 1.0);
	p.param<bool>("active_hibrid", ACTIVE_HIBRID, true);
	p.param<double>("newer_duplication_threshold", NEWER_DUPLICATION_THRESHOLD, 100);
	p.param<double>("branch_tolerance", BRANCH_TOLERANCE, 1.0);
	
}

bool BranchSearch::getGoal(geometry_msgs::Point& goal){
	if(pose_.q.callOne(ros::WallDuration(1)) || scan_.q.callOne(ros::WallDuration(1))) return false;

	const int scanWidth = BRANCH_ANGLE / scan_.data.angle_increment;
    const int scanMin = (scan_.data.ranges.size()/2)-1 - scanWidth;
    const int scanMax = (scan_.data.ranges.size()/2) + scanWidth;

    for(int i = scanMin;i!=scanMax;++i){
		if(!std::isnan(scan_.data.ranges[i]) && scan_.data.ranges[i] < CENTER_RANGE_MIN){
			ROS_ERROR_STREAM("It may be Close to Obstacles");
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
		ROS_ERROR_STREAM("scan_.data is Insufficient");
		return false;
    }

	if(branchDetection(scanRect,goal,pose_.data.pose)){
		publishGoal(goal);
		ROS_INFO_STREAM("Branch Found : (" << goal.x << "," << goal.y << ")");
		return true;
    }
	else{
		ROS_INFO_STREAM("Branch Do Not Found");
		return false;
	}
}

bool BranchSearch::branchDetection(const CommonLib::scanStruct& ss,geometry_msgs::Point& goal,const geometry_msgs::Pose& pose){
	ROS_DEBUG_STREAM("Searching Branch");

	//なんフレームか連続で発見できないとダメということにする？

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
		if(ss.ranges[i] < SCAN_RANGE_THRESHOLD && ss.ranges[i+1] < SCAN_RANGE_THRESHOLD){//距離が遠いのは信用できないのでだめ
			double scanX = ss.ranges[i]*cos(ss.angles[i]);
			double nextScanX = ss.ranges[i+1]*cos(ss.angles[i+1]);
			if(std::abs(nextScanX - scanX) >= BRANCH_DIFF_X_MIN){//x座標の差(分岐の幅)が一定以上じゃないと分岐と認めないフィルタ
				double scanY = ss.ranges[i]*sin(ss.angles[i]);
				double nextScanY = ss.ranges[i+1]*sin(ss.angles[i+1]);
				double diffY = std::abs(nextScanY - scanY);
				if(BRANCH_MIN_Y <= diffY && diffY <= BRANCH_MAX_Y){//分岐のy座標の差は一定の範囲に入っていないと分岐にしないフィルタ
					localList.emplace_back(CommonLib::msgPoint((nextScanX + scanX)/2,(nextScanY + scanY)/2));
				}
			}
		}
	}

	
	if(localList.size()>0){
		ROS_DEBUG_STREAM("Branch Candidate Found : " << localList.size());

		double yaw = CommonLib::qToYaw(pose.orientation);

		// std::vector<geometry_msgs::Point> globalList;
		std::vector<listStruct> globalList;
		globalList.reserve(localList.size());
		
		for(const auto& l : localList) globalList.emplace_back(CommonLib::msgPoint(pose.position.x+(cos(yaw)*l.x)-(sin(yaw)*l.y),pose.position.y+(cos(yaw)*l.y)+(sin(yaw)*l.x)));

		//ここで前の目標と近いやつはリストから削除
		static geometry_msgs::Point lastBranch;
			
		std::vector<listStruct> tempList;
		tempList.reserve(globalList.size());
		for(const auto& g : globalList){
			if(Eigen::Vector2d(g.point.x - lastBranch.x,g.point.y - lastBranch.y).norm()>BRANCH_TOLERANCE) tempList.emplace_back(g);
		}
		if(tempList.size() == 0) return false;
		
		globalList = std::move(tempList);

		publishGoalArray(listStructToPoint(globalList));

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
					goal = globalList[j].point;
					id = j;
				}
			}

			ROS_DEBUG_STREAM("Branch Candidate : (" << goal.x << "," << goal.y << ")");

			if(DUPLICATE_CHECK){
				switch (duplicateDetection(goal)){
					case NOT_DUPLECATION:
						return true;
					case OLDER://重複してるけど古いからフロンティアも見ておく
						globalList[id].duplication = OLDER;
						break;
					case NEWER://重複かつ最近通ったところなのでフロンティアも見ない
						globalList[id].duplication = NEWER;
						break;
				}
				excludeList.push_back(id);
			}
			// if(DUPLICATE_CHECK && duplicateDetection(goal)) excludeList.push_back(id);
			else {
				lastBranch = goal;
				return true;
			}
		}

		//直前に通ったばかりの分岐は強めに通っては行けない

		//グローバルリストとフロンティア領域を比較して重複してても曲がるべきかを判断
		//残っているフロンティアに対してアクセスしやすい方向に進みたいので、角度の総和が小さい方が良い <- これ決定
		if(ACTIVE_HIBRID){
			std::vector<exploration_msgs::Frontier> frontiers(fs.frontierDetection<std::vector<exploration_msgs::Frontier>>(false));
			
			if(frontiers.size()!=0){
				
				for(const auto& f : frontiers) {
					ROS_DEBUG_STREAM("frontier : " << f.coordinate << ", " << f.variance << ", " << f.covariance);
				}
				double min = DBL_MAX;
				for(const auto& g : globalList){
					//重複判定がNEWERだったらスキップ
					if(g.duplication == NEWER){
						ROS_INFO_STREAM("newer duplication!!");
						continue;
					}
					//座標がthroughBranchに入ってたらスキップ//重複探査阻止の処理を回避するのは二回以上出来ない
					bool through = false;
					for(const auto& t : throughBranch){
						if(Eigen::Vector2d(g.point.x-t.x,g.point.y-t.y).norm() < THROUGH_TOLERANCE){
							through = true;
							ROS_DEBUG_STREAM("through branch");
							break;
						}
					}
					if(through) continue;
					double val = fs.evoluatePointToFrontier(g.point,Eigen::Vector2d(g.point.x-pose.position.x,g.point.y-pose.position.y).normalized(),frontiers);
					if(min > val){
						min = std::move(val);
						goal = g.point;
						lastBranch = goal;
					}
				}
				//最後に直進方向のフロンティア面積と比較する //逆方向に行った時の奴も比較したほうが良いかも
				if(fs.evoluatePointToFrontier(pose,Eigen::Vector2d(goal.x-pose.position.x,goal.y-pose.position.y).norm(),frontiers) > min){
					ROS_DEBUG_STREAM("Branch : (" << goal.x << "," << goal.y << ")");
					ROS_DEBUG_STREAM("This Branch continues to a large frontier");
					throughBranch.emplace_back(goal);
					return true;
				}
			}
		}
		
    }

	return false;
}

BranchSearch::Duplication BranchSearch::duplicateDetection(const geometry_msgs::Point& goal){
	if(poseLog_.q.callOne(ros::WallDuration(1))) return NOT_DUPLECATION;
	//重複探査の新しさとかはヘッダーの時間で見る
	//重複が新しいときと古い時で挙動を変える
	
	//重複探査を考慮する時間の上限から参照する配列の最大値を設定
	int ARRAY_MAX = poseLog_.data.poses.size();
	for(int i=poseLog_.data.poses.size()-2;i!=0;--i){
		if(ros::Duration(poseLog_.data.header.stamp - poseLog_.data.poses[i].header.stamp).toSec() > LOG_NEWER_LIMIT){
			ARRAY_MAX = i;
			break;
		}
	}

	for(int i=ARRAY_MAX;i!=0;--i){
	// for(int i=0,e=poseLog_.data.poses.size();i!=e;++i){
		//過去のオドメトリが重複判定の範囲内に入っているか//
		if(Eigen::Vector2d(goal.x-poseLog_.data.poses[i].pose.position.x,goal.y-poseLog_.data.poses[i].pose.position.y).norm() < DUPLICATE_TOLERANCE){
			ROS_DEBUG_STREAM("This Branch is Duplicated");
			return ros::Duration(poseLog_.data.header.stamp - poseLog_.data.poses[i].header.stamp).toSec() > NEWER_DUPLICATION_THRESHOLD ? OLDER : NEWER;
		}
	}
	ROS_DEBUG_STREAM("This Branch is Not Duplicated");
	return NOT_DUPLECATION;
}

std::vector<geometry_msgs::Point> BranchSearch::listStructToPoint(const std::vector<listStruct>& list){
	std::vector<geometry_msgs::Point> point;
	point.reserve(list.size());
	for(const auto& l : list) point.emplace_back(l.point);
	return point;
}

void BranchSearch::publishGoal(const geometry_msgs::Point& goal){
	geometry_msgs::PointStamped msg;
	msg.point = goal;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = MAP_FRAME_ID;
	goal_.pub.publish(msg);
	ROS_INFO_STREAM("Publish Goal");
}

void BranchSearch::publishGoalArray(const std::vector<geometry_msgs::Point>& goals){
	exploration_msgs::PointArray msg;
	msg.points = goals;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = MAP_FRAME_ID;
	goalArray_.pub.publish(msg);
	ROS_INFO_STREAM("Publish GoalList");
}

#endif //BRANCH_SEARCH_HPP