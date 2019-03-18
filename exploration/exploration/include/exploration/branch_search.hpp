#ifndef BRANCH_SEARCH_H
#define BRANCH_SEARCH_H

//分岐領域を検出するクラス
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <exploration_msgs/Goal.h>
#include <exploration_msgs/GoalList.h>
#include <std_msgs/Empty.h>

//分岐領域を検出
class BranchSearch
{
private:
	//分岐検出用パラメータ
	ros::NodeHandle p;
    double BRANCH_ANGLE;
    double CENTER_RANGE_MIN;
	double BRANCH_DIST_LIMIT;
	double BRANCH_DIFF_X_MIN;
	double DUPLICATE_MARGIN;
	bool DUPLICATE_CHECK;
	double DOUBLE_INFINITY;

	ros::NodeHandle spl;
    ros::Subscriber subPoseLog;
    ros::CallbackQueue qPoseLog;
	geometry_msgs::PoseArray poseLogData;

	ros::NodeHandle pg;
	ros::Publisher pubGoal;

	ros::NodeHandle pgd;
	ros::Publisher pubGoalDel;

	ros::NodeHandle pgl;
	ros::Publisher pubGoalList;

	ros::NodeHandle pgld;
	ros::Publisher pubGoalListDel;

	ros::NodeHandle ss;
    ros::Subscriber subScan;
    ros::CallbackQueue qScan;
    sensor_msgs::LaserScan scanData;

	ros::NodeHandle sp;
    ros::Subscriber subPose;
    ros::CallbackQueue qPose;
    geometry_msgs::PoseStamped poseData;

	std::string mapFrameId;

	void scanCB(const sensor_msgs::LaserScan::ConstPtr& msg);
	void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void poseLogCB(const geometry_msgs::PoseArray::ConstPtr& msg);

	geometry_msgs::Point getGoalBranch(const sensor_msgs::LaserScan& scan,const geometry_msgs::PoseStamped& pose);
	bool branchDetection(const std::vector<float>& ranges, const std::vector<float>& angles,float angleMax,geometry_msgs::Point& goal,geometry_msgs::Point& localGoal,const geometry_msgs::PoseStamped& pose);
	double qToYaw(const geometry_msgs::Quaternion& q);
    bool duplicateDetection(const geometry_msgs::Point& goal);
	void publishGoal(const geometry_msgs::Point& global, const geometry_msgs::Point& local);
	void publishGoalList(const std::vector<geometry_msgs::Point>& global, const std::vector<geometry_msgs::Point>& local);

	void publishGoalDelete(void);
	void publishGoalListDelete(void);

public:
    BranchSearch();
    ~BranchSearch(){};

	bool getGoal(geometry_msgs::Point& goal);
};

BranchSearch::BranchSearch():p("~"){
    ss.setCallbackQueue(&qScan);
    subScan = ss.subscribe("scan",1,&BranchSearch::scanCB, this);

    sp.setCallbackQueue(&qPose);
    subPose = sp.subscribe("pose",1,&BranchSearch::poseCB,this);

    spl.setCallbackQueue(&qPoseLog);
    subPoseLog = spl.subscribe("pose_log",1,&BranchSearch::poseLogCB, this);

	pubGoal = pg.advertise<exploration_msgs::Goal>("goal", 1, true);
	pubGoalList = pgl.advertise<exploration_msgs::GoalList>("goal_list", 1, true);
	pubGoalDel = pgd.advertise<std_msgs::Empty>("goal/delete", 1);
	pubGoalListDel = pgld.advertise<std_msgs::Empty>("goal_list/delete", 1);

	p.param<std::string>("map_frame_id", mapFrameId, "map");

	//branch_searchパラメータの読み込み(基本変更しなくて良い)

	p.param<double>("branch_angle", BRANCH_ANGLE, 0.04);
	p.param<double>("center_range_min", CENTER_RANGE_MIN, 1.0);
	p.param<double>("branch_range_limit", BRANCH_DIST_LIMIT, 5.0);
	p.param<double>("branch_diff_x_min", BRANCH_DIFF_X_MIN, 1.0);
	p.param<double>("duplicate_margin", DUPLICATE_MARGIN, 1.5);
	p.param<bool>("duplicate_check", DUPLICATE_CHECK, true);

	DOUBLE_INFINITY = 10000.0;
}

void BranchSearch::scanCB(const sensor_msgs::LaserScan::ConstPtr& msg){
        scanData = *msg;
}

void BranchSearch::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
        poseData = *msg;
}

void BranchSearch::poseLogCB(const geometry_msgs::PoseArray::ConstPtr& msg){
	poseLogData = *msg;
}

bool BranchSearch::getGoal(geometry_msgs::Point& goal){
	qPose.callOne(ros::WallDuration(1));
	qScan.callOne(ros::WallDuration(1));

	publishGoalDelete();
	publishGoalListDelete();

	goal = getGoalBranch(scanData,poseData);

	if((int)goal.x == 0 && (int)goal.y == 0 && (int)goal.z == 0){
		ROS_INFO_STREAM("Branch Do Not Found\n");
        return false;
    }
    else{
		ROS_INFO_STREAM("Branch Found : (" << goal.x << "," << goal.y << ")\n");
        return true;
    }
}

geometry_msgs::Point BranchSearch::getGoalBranch(const sensor_msgs::LaserScan& scan, const geometry_msgs::PoseStamped& pose){
    const int scanWidth = BRANCH_ANGLE / scan.angle_increment;
    const int scanMin = (scan.ranges.size()/2)-1 - scanWidth;
    const int scanMax = (scan.ranges.size()/2) + scanWidth;

    geometry_msgs::Point goal;
	geometry_msgs::Point localGoal;

    for(int i = scanMin;i<scanMax;i++){
		if(!std::isnan(scan.ranges[i]) && scan.ranges[i] < CENTER_RANGE_MIN){
			ROS_ERROR_STREAM("It may be Close to Obstacles\n");
			return goal;
		}
    }

    std::vector<float> fixRanges;
    std::vector<float> fixAngles;

    for(int i=0;i<scan.ranges.size();i++){
		if(!std::isnan(scan.ranges[i])){
			fixRanges.push_back(scan.ranges[i]);
			fixAngles.push_back(scan.angle_min+(scan.angle_increment*i));
		}
    }

    if(fixRanges.size() < 2){
		ROS_ERROR_STREAM("ScanData is Insufficient\n");
		return goal;
    }

    if(branchDetection(fixRanges,fixAngles,scan.angle_max,goal,localGoal,pose)){
		publishGoal(goal,localGoal);
    }
	else{
		geometry_msgs::Point temp;
        goal = temp;
	}

    return goal;
}

bool BranchSearch::branchDetection(const std::vector<float>& ranges, const std::vector<float>& angles,float angleMax,geometry_msgs::Point& goal,geometry_msgs::Point& localGoal,const geometry_msgs::PoseStamped& pose){

	ROS_DEBUG_STREAM("Searching Branch\n");

	float scanX,scanY,nextScanX,nextScanY;
	float diffX,diffY;

	//分岐のy座標の差がこの値の範囲内の場合のみ分岐として検出
	const float BRANCH_MIN_Y = BRANCH_DIFF_X_MIN*tan(angleMax);
	const float BRANCH_MAX_Y = BRANCH_DIST_LIMIT*tan(angleMax);

	std::vector<geometry_msgs::Point> list;
	geometry_msgs::Point tempGoal;

    //はじめのfor文では条件を満たした分岐領域を探す
    //２つ目のfor文で最も近い分岐を選ぶ
    //選んだ最も近い分岐に対して重複探査の確認を行いtrueが帰ってきたらその分岐を除いて再度最も近い分岐を探す

	for(int i=0;i<ranges.size()-1;i++){
		scanX = ranges[i]*cos(angles[i]);
		nextScanX = ranges[i+1]*cos(angles[i+1]);
		if(scanX <= BRANCH_DIST_LIMIT && nextScanX <= BRANCH_DIST_LIMIT){
			diffX = std::abs(nextScanX - scanX);
			if(diffX >= BRANCH_DIFF_X_MIN){
				scanY = ranges[i]*sin(angles[i]);
				nextScanY = ranges[i+1]*sin(angles[i+1]);
				diffY = std::abs(nextScanY - scanY);
				if(BRANCH_MIN_Y <= diffY && diffY <= BRANCH_MAX_Y){
					tempGoal.x = (nextScanX + scanX)/2;
					tempGoal.y = (nextScanY + scanY)/2;
					list.push_back(tempGoal);
				}
			}
		}
	}
	
    bool find = false;

	if(list.size()>0){
		ROS_DEBUG_STREAM("Branch Candidate Found\n");

		int near;
		float centerDist;
        bool tempCenterDist;
		double yaw = qToYaw(pose.pose.orientation);

		std::vector<geometry_msgs::Point> globalList;
		globalList.resize(list.size());
		geometry_msgs::Point tempGlobal;

		for(int i=0;i<list.size();i++){
			tempGlobal.x = pose.pose.position.x + (cos(yaw)*list[i].x) - (sin(yaw)*list[i].y);
			tempGlobal.y = pose.pose.position.y + (cos(yaw)*list[i].y) + (sin(yaw)*list[i].x);
			globalList[i] = tempGlobal;
		}

		publishGoalList(globalList,list);

		for(int k=list.size();k>0;k--){
			centerDist = DOUBLE_INFINITY;
			for(int j=0;j<list.size();j++){
				tempCenterDist = std::abs(list[j].x)+std::abs(list[j].y);
				if(tempCenterDist <= centerDist){
					centerDist = tempCenterDist;
					goal = globalList[j];
					localGoal = list[j];
					near = j;
				}
			}

			ROS_DEBUG_STREAM("Branch Candidate : (" << goal.x << "," << goal.y << ")\n");

			if(DUPLICATE_CHECK && duplicateDetection(goal)){
				list.erase(list.begin() + near);
				find = false;
			}
			else{
				find = true;
				break;
			}
		}	
    }
    return find;
}

double BranchSearch::qToYaw(const geometry_msgs::Quaternion& q){
    tf::Quaternion tq(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(tq).getRPY(roll,pitch,yaw);
    return yaw;
}

bool BranchSearch::duplicateDetection(const geometry_msgs::Point& goal){
	double globalX;//分岐領域の世界座標
	double globalY;//分岐領域の世界座標
	double xPlus;
	double xMinus;
	double yPlus;
	double yMinus;
	const int lOG_NEWER_LIMIT = 30;//ログの取得が1Hzの場合30秒前までのログで重複検出

	qPoseLog.callOne(ros::WallDuration(1));

	xPlus = goal.x + DUPLICATE_MARGIN;
	xMinus = goal.x - DUPLICATE_MARGIN;
	yPlus = goal.y + DUPLICATE_MARGIN;
	yMinus = goal.y - DUPLICATE_MARGIN;

	for(int i=0;i<poseLogData.poses.size()-lOG_NEWER_LIMIT;i++){
		//過去のオドメトリが許容範囲の中に入っているか//
		if((xPlus >= poseLogData.poses[i].position.x) && (xMinus <= poseLogData.poses[i].position.x)){
			if((yPlus >= poseLogData.poses[i].position.y) && (yMinus <= poseLogData.poses[i].position.y)){
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
	msg.header.frame_id = mapFrameId;

	pubGoal.publish(msg);
	ROS_INFO_STREAM("Publish Goal\n");
}

void BranchSearch::publishGoalList(const std::vector<geometry_msgs::Point>& global, const std::vector<geometry_msgs::Point>& local){
	exploration_msgs::GoalList msg;
	msg.global = global;
	msg.local = local;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = mapFrameId;

	pubGoalList.publish(msg);
	ROS_INFO_STREAM("Publish GoalList\n");
}

void BranchSearch::publishGoalDelete(void){
	std_msgs::Empty msg;
	pubGoalDel.publish(msg);
	
	//ROS_INFO_STREAM("Publish Goal Delete\n");
}

void BranchSearch::publishGoalListDelete(void){
	std_msgs::Empty msg;
	pubGoalListDel.publish(msg);
	
	//ROS_INFO_STREAM("Publish GoalList Delete\n");
}

#endif //BRANCH_SEARCH_H