#ifndef BRANCH_SEARCH_HPP
#define BRANCH_SEARCH_HPP

//分岐領域を検出するクラス
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <exploration_msgs/Goal.h>
#include <exploration_msgs/GoalList.h>
#include <std_msgs/Empty.h>
#include <exploration/common_lib.hpp>
#include <exploration/frontier_search.hpp>

//分岐領域を検出
class BranchSearch
{
private:
	
	//パラメータ
	ros::NodeHandle p;
    double BRANCH_ANGLE;
    double CENTER_RANGE_MIN;
	double BRANCH_MAX_X;
	double BRANCH_DIFF_X_MIN;
	double DUPLICATE_TOLERANCE;
	bool DUPLICATE_CHECK;
	double DOUBLE_INFINITY;
	std::string MAP_FRAME_ID;

	CommonLib::subStruct<geometry_msgs::PoseArray> poseLog_;
	CommonLib::subStruct<sensor_msgs::LaserScan> scan_;
	CommonLib::subStruct<geometry_msgs::PoseStamped> pose_;

	CommonLib::pubStruct<exploration_msgs::Goal> goal_;
	CommonLib::pubStruct<exploration_msgs::GoalList> goalList_;

	FrontierSearch fs;

	bool branchDetection(const CommonLib::scanStruct& ss,geometry_msgs::Point& goal,geometry_msgs::Point& localGoal,const geometry_msgs::Pose& pose);
    bool duplicateDetection(const geometry_msgs::Point& goal);
	void publishGoal(const geometry_msgs::Point& global, const geometry_msgs::Point& local);
	void publishGoalList(const std::vector<geometry_msgs::Point>& global, const std::vector<geometry_msgs::Point>& local);


public:
    BranchSearch();
	bool getGoal(geometry_msgs::Point& goal);
};

BranchSearch::BranchSearch()
	:p("~")
	,poseLog_("pose_log",1)
	,scan_("scan",1)
	,pose_("pose",1)
	,goal_("goal", 1, true)
	,goalList_("goal_list", 1, true)
	,DOUBLE_INFINITY(10000.0){

	//branch_searchパラメータの読み込み(基本変更しなくて良い)
	p.param<std::string>("map_frame_id", MAP_FRAME_ID, "map");
	p.param<double>("branch_angle", BRANCH_ANGLE, 0.04);
	p.param<double>("center_range_min", CENTER_RANGE_MIN, 1.0);
	p.param<double>("branch_max_x", BRANCH_MAX_X, 6.0);
	p.param<double>("branch_diff_x_min", BRANCH_DIFF_X_MIN, 1.0);
	p.param<double>("duplicate_tolerance", DUPLICATE_TOLERANCE, 1.5);
	p.param<bool>("duplicate_check", DUPLICATE_CHECK, true);
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

	//ここの

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

	geometry_msgs::Point localGoal;

	if(branchDetection(scanRect,goal,localGoal,pose_.data.pose)){
		//ROS_DEBUG_STREAM("debug1\n");
		publishGoal(goal,localGoal);
		ROS_INFO_STREAM("Branch Found : (" << goal.x << "," << goal.y << ")\n");
		return true;
    }
	else{
		ROS_INFO_STREAM("Branch Do Not Found\n");
		return false;
	}
}

bool BranchSearch::branchDetection(const CommonLib::scanStruct& ss,geometry_msgs::Point& goal,geometry_msgs::Point& localGoal,const geometry_msgs::Pose& pose){
	ROS_DEBUG_STREAM("Searching Branch\n");

	float scanX,scanY,nextScanX,nextScanY;
	float diffX,diffY;

	//壁として見えたものが一定以上の長さで続いていないとダメ//点や人が少し通っただけで分岐になるとうっとおしい
	//ダメっぽいので一定フレームの間存在したら

	//分岐のy座標の差がこの値の範囲内の場合のみ分岐として検出
	const float BRANCH_MIN_Y = BRANCH_DIFF_X_MIN*tan(ss.angleMax);//1.0 * tan(0.52) = 0.57
	const float BRANCH_MAX_Y = BRANCH_MAX_X*tan(ss.angleMax);//5.0 * tan(0.52) = 2.86 //この二つの差が正しいのでは // 6.0/tan(1.05)

	std::vector<geometry_msgs::Point> list;
	list.reserve(ss.ranges.size());

    //はじめのfor文では条件を満たした分岐領域を探す
    //２つ目のfor文で最も近い分岐を選ぶ
    //選んだ最も近い分岐に対して重複探査の確認を行いtrueが帰ってきたらその分岐を除いて再度最も近い分岐を探す

	//重複探査で全部弾かれた場合でもフロンティア領域との関係を見て

	//角度が正側と負側で処理を分ける
	//マイナス

	const double yLengthThreshold = 0.75;

	//xyのmaxではなくrangesのmaxで制限しないと意味ないのでは
	//ロボットが壁に垂直な状態でしか

	for(int i=0,e=ss.ranges.size()-1;i!=e;++i){
		//二つの角度の符号が違うときスキップ
		if(ss.angles[i]*ss.angles[i+1]<0){
			continue;
		}
		scanX = ss.ranges[i]*cos(ss.angles[i]);
		nextScanX = ss.ranges[i+1]*cos(ss.angles[i+1]);
		if(scanX <= BRANCH_MAX_X && nextScanX <= BRANCH_MAX_X){//距離が遠い分岐は信用できないフィルタ
			diffX = std::abs(nextScanX - scanX);
			if(diffX >= BRANCH_DIFF_X_MIN){//x座標の差(分岐の幅)が一定以上じゃないと分岐と認めないフィルタ
				scanY = ss.ranges[i]*sin(ss.angles[i]);
				nextScanY = ss.ranges[i+1]*sin(ss.angles[i+1]);
				diffY = std::abs(nextScanY - scanY);
				if(BRANCH_MIN_Y <= diffY && diffY <= BRANCH_MAX_Y){//分岐のy座標の差は一定の範囲に入っていないと分岐にしないフィルタ
					double dxRight = ss.ranges[i]*cos(ss.angles[i]) - ss.ranges[0]*cos(ss.angles[0]);
					double dyRight = ss.ranges[i]*sin(ss.angles[i]) - ss.ranges[0]*sin(ss.angles[0]);
					double yLengthRight = sqrt(dxRight*dxRight+dyRight*dyRight);

					double dxLeft = ss.ranges[i+1]*cos(ss.angles[i+1]) - ss.ranges[e]*cos(ss.angles[e]);
					double dyLeft = ss.ranges[i+1]*sin(ss.angles[i+1]) - ss.ranges[e]*sin(ss.angles[e]);
					double yLengthLeft = sqrt(dxLeft*dxLeft+dyLeft*dyLeft);

					ROS_DEBUG_STREAM("yLengthRight : " << yLengthRight << ", yLengthLeft : " << yLengthLeft << "\n");
					ROS_DEBUG_STREAM("diffX : " << diffX << ", diffY : " << diffY << "\n");

					//やはり前後にいくつ続いているかを見たほうが良い
					


					if(yLengthLeft > yLengthThreshold && yLengthRight > yLengthThreshold){//見つけた分岐から左右に一定以上センサデータが続いていないと分岐にしないフィルタ
						list.emplace_back(CommonLib::msgPoint((nextScanX + scanX)/2,(nextScanY + scanY)/2));
					}
				}
			}
		}
	}
	
	if(list.size()>0){
		ROS_DEBUG_STREAM("Branch Candidate Found : " << list.size() << "\n");

		int near;
		float centerDist;
		double yaw = CommonLib::qToYaw(pose.orientation);

		std::vector<geometry_msgs::Point> globalList;
		globalList.reserve(list.size());
		
		for(const auto& branch : list){
			globalList.emplace_back(CommonLib::msgPoint(pose.position.x + (cos(yaw)*branch.x) - (sin(yaw)*branch.y),pose.position.y + (cos(yaw)*branch.y) + (sin(yaw)*branch.x)));
		}

		publishGoalList(globalList,list);

		std::vector<int> excludeList;
		excludeList.reserve(list.size());

		for(int k=list.size();k!=0;--k){
			centerDist = DOUBLE_INFINITY;
			for(int j=0,e=list.size();j!=e;++j){
				if(excludeList.size()>0 && std::any_of(excludeList.begin(), excludeList.end(), [j](int n){ return n == j; })){
					continue;
				}
				float tempCenterDist = std::abs(list[j].x)+std::abs(list[j].y);
				if(tempCenterDist <= centerDist){
					centerDist = std::move(tempCenterDist);
					goal = globalList[j];
					localGoal = list[j];
					near = j;
				}
			}

			ROS_DEBUG_STREAM("Branch Candidate : (" << goal.x << "," << goal.y << ")\n");

			if(DUPLICATE_CHECK && duplicateDetection(goal)){
				excludeList.push_back(near);
				// list[near].x = 0;
				// list[near].y = 0;
			}
			else{
				return true;
			}
		}

		//グローバルリストとフロンティア領域を比較して重複してても曲がるべきかを判断
		//残っているフロンティアに対してアクセスしやすい方向に進みたいので、角度の総和が小さい方が良い <- これ決定
		std::vector<geometry_msgs::Point> frontiers(fs.frontierDetection(false));
		if(frontiers.size()!=0){
			double min = DOUBLE_INFINITY;
			for(int i=0,ei=globalList.size();i!=ei;++i){
				double sum = fs.sumFrontierAngle(globalList[i],Eigen::Vector2d(globalList[i].x-pose.position.x,globalList[i].y-pose.position.y).normalized(),frontiers);
				if(min > sum){
					min = std::move(sum);
					goal = globalList[i];
					localGoal = list[i];
				}
			}
			//最後に直進方向のフロンティア面積と比較する
			if(fs.sumFrontierAngle(pose,BRANCH_MAX_X,frontiers) > min){
				ROS_DEBUG_STREAM("Branch : (" << goal.x << "," << goal.y << ")\n");
				ROS_DEBUG_STREAM("This Branch continues to a large frontier\n");
				return true;
			}
		}
		
    }


	return false;
}

bool BranchSearch::duplicateDetection(const geometry_msgs::Point& goal){

	const int lOG_NEWER_LIMIT = 30;//ログの取得が1Hzの場合30秒前までのログで重複検出

	poseLog_.q.callOne(ros::WallDuration(1));

	double xPlus = goal.x + DUPLICATE_TOLERANCE;
	double xMinus = goal.x - DUPLICATE_TOLERANCE;
	double yPlus = goal.y + DUPLICATE_TOLERANCE;
	double yMinus = goal.y - DUPLICATE_TOLERANCE;

	//ROS_DEBUG_STREAM("duplicate log size : " << poseLog_.data.poses.size() << "\n");

	for(int i=0,e=poseLog_.data.poses.size()-lOG_NEWER_LIMIT;i!=e;++i){
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

void BranchSearch::publishGoal(const geometry_msgs::Point& global, const geometry_msgs::Point& local){
	exploration_msgs::Goal msg;
	msg.global = global;
	msg.local = local;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = MAP_FRAME_ID;
	goal_.pub.publish(msg);
	ROS_INFO_STREAM("Publish Goal\n");
}

void BranchSearch::publishGoalList(const std::vector<geometry_msgs::Point>& global, const std::vector<geometry_msgs::Point>& local){
	exploration_msgs::GoalList msg;
	msg.global = global;
	msg.local = local;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = MAP_FRAME_ID;

	goalList_.pub.publish(msg);
	ROS_INFO_STREAM("Publish GoalList\n");
}

#endif //BRANCH_SEARCH_HPP