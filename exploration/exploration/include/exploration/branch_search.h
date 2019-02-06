#ifndef BRANCH_SEARCH_H
#define BRANCH_SEARCH_H

//分岐領域を検出するクラス
#include <ros/ros.h>
//#include <ros/console.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <exploration_msgs/PointArray.h>
//#include <sensor_based_exploration/PoseLog.h>

//分岐領域を検出
class BranchSearch
{
private:
	//分岐検出用パラメータ
	ros::NodeHandle p;
    double BRANCH_ANGLE;
    double CENTER_RANGE_MIN;
    //double BRANCH_RANGE_LIMIT;
	double BRANCH_DIST_LIMIT;
    //double BRANCH_DIFF_THRESHOLD;
	double BRANCH_DIFF_X_MIN;
	double DUPLICATE_MARGIN;

	bool DUPLICATE_CHECK;

	double DOUBLE_INFINITY;

	ros::NodeHandle spl;
    ros::Subscriber subPoseLog;
    ros::CallbackQueue qPoseLog;
    //std::string poseLogTopic;
    //std::vector<geometry_msgs::PoseStamped> poseLogData;
	geometry_msgs::PoseArray poseLogData;

	ros::NodeHandle pg;
	ros::Publisher pubGoal;
	//std::string goalTopic;

	ros::NodeHandle pgl;
	ros::Publisher pubGoalList;
	//std::string goalListTopic;

	ros::NodeHandle ss;
    ros::Subscriber subScan;
    ros::CallbackQueue qScan;
    //std::string scanTopic;
    sensor_msgs::LaserScan scanData;

	ros::NodeHandle sp;
    ros::Subscriber subPose;
    ros::CallbackQueue qPose;
    //std::string poseTopic;
    geometry_msgs::PoseStamped poseData;

	std::string mapFrameId;

	void scanCB(const sensor_msgs::LaserScan::ConstPtr& msg);
	void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void poseLogCB(const geometry_msgs::PoseArray::ConstPtr& msg);

	geometry_msgs::Point getGoalBranch(sensor_msgs::LaserScan scan,geometry_msgs::PoseStamped pose);
	bool branchDetection(std::vector<float>& ranges, std::vector<float>& angles,float angleMax,geometry_msgs::Point& goal,geometry_msgs::PoseStamped pose);
    bool duplicateDetection(geometry_msgs::Point goal);
	void publishGoal(geometry_msgs::Point goal);
	void publishGoalList(std::vector<geometry_msgs::Point> list);

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

	pubGoal = pg.advertise<geometry_msgs::PointStamped>("goal", 1);

	pubGoalList = pgl.advertise<exploration_msgs::PointArray>("goal_list", 1);

	// p.param<std::string>("scan_topic", scanTopic, "scan");
    // ss.setCallbackQueue(&qScan);
    // subScan = ss.subscribe(scanTopic,1,&BranchSearch::scanCB, this);

	// p.param<std::string>("pose_topic", poseTopic, "pose");
    // sp.setCallbackQueue(&qPose);
    // subPose = sp.subscribe(poseTopic,1,&BranchSearch::poseCB,this);

	// p.param<std::string>("pose_log_topic", poseLogTopic, "pose_log");
    // spl.setCallbackQueue(&qPoseLog);
    // subPoseLog = spl.subscribe(poseLogTopic,1,&BranchSearch::poseLogCB, this);

	// p.param<std::string>("goal_topic", goalTopic, "goal");
	// pubGoal = pg.advertise<geometry_msgs::PointStamped>(goalTopic, 1);

	// p.param<std::string>("goal_list_topic", goalListTopic, "goal_list");
	// pubGoalList = pgl.advertise<exploration_msgs::PointArray>(goalListTopic, 1);


	p.param<std::string>("map_frame_id", mapFrameId, "map");

	//branch_searchパラメータの読み込み(基本変更しなくて良い)

	p.param<double>("branch_angle", BRANCH_ANGLE, 0.04);
	p.param<double>("center_range_min", CENTER_RANGE_MIN, 1.0);
	//p.param<double>("branch_range_limit", BRANCH_RANGE_LIMIT, 1.0);
	p.param<double>("branch_range_limit", BRANCH_DIST_LIMIT, 5.0);
	//p.param<double>("branch_diff_threshold", BRANCH_DIFF_THRESHOLD, 1.0);
	p.param<double>("branch_diff_x_min", BRANCH_DIFF_X_MIN, 1.0);
	p.param<double>("duplicate_margin", DUPLICATE_MARGIN, 1.5);
	p.param<bool>("duplicate_check", DUPLICATE_CHECK, true);

	DOUBLE_INFINITY = 10000.0;
}

void BranchSearch::scanCB(const sensor_msgs::LaserScan::ConstPtr& msg){
        scanData = *msg;
        // if(scanEdit){
        //     approx(scanData.ranges);
        // }
}

void BranchSearch::poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
        poseData = *msg;
}

// void BranchSearch::initialize(double branchAngle, double centerRangeMin){
//     BRANCH_ANGLE = branchAngle;
//     CENTER_RANGE_MIN = centerRangeMin;
//     //BRANCH_RANGE_LIMIT = 
//     //BRANCH_DIFF_THRESHOLD
// 	//ROBOT_NAME = 
// 	//DUPLICATE_MARGIN
// }

//void BranchSearch::poseLogCB(const sensor_based_exploration::PoseLog::ConstPtr& msg){
void BranchSearch::poseLogCB(const geometry_msgs::PoseArray::ConstPtr& msg){
	// for(int i=0;i<msg -> names.size();i++){
	// 	if(msg->names[i] == ROBOT_NAME){
	// 		poseLogData = msg->poseLists[i].poses;
	// 		break;
	// 	}
	// }
	poseLogData = *msg;
}

bool BranchSearch::getGoal(geometry_msgs::Point& goal){
	qPose.callOne(ros::WallDuration(1));
	qScan.callOne(ros::WallDuration(1));

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

geometry_msgs::Point BranchSearch::getGoalBranch(sensor_msgs::LaserScan scan, geometry_msgs::PoseStamped pose){
    const int scanWidth = BRANCH_ANGLE / scan.angle_increment;
    const int scanMin = (scan.ranges.size()/2)-1 - scanWidth;
    const int scanMax = (scan.ranges.size()/2) + scanWidth;

    geometry_msgs::Point goal;

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

    //bool branchFlag;

    //branchFlag = branchDetection(fixRanges,fixAngle,scanData.angle_max,goal);
    if(branchDetection(fixRanges,fixAngles,scan.angle_max,goal,pose)){
		//double yaw = 2*asin(pose.pose.orientation.z);
		//goal.x = pose.pose.position.x+(cos(yaw)*goal.x) - (sin(yaw)*goal.y);
		//globalY = pose.pose.position.y+(cos(yaw)*goal.y) + (sin(yaw)*goal.x);
		publishGoal(goal);
    }
	else{
		geometry_msgs::Point temp;
        goal = temp;
	}
	//publishGoalBranch(goal);

    return goal;
}

bool BranchSearch::branchDetection(std::vector<float>& ranges, std::vector<float>& angles,float angleMax,geometry_msgs::Point& goal,geometry_msgs::PoseStamped pose){

	ROS_DEBUG_STREAM("Searching Branch\n");

	float scanX,scanY,nextScanX,nextScanY;
	float diffX,diffY;

	//float Branch_y_dist;

	//分岐のy座標の差がこの値の範囲内の場合のみ分岐として検出
	//const float BRANCH_LOW_Y = BRANCH_DIFF_THRESHOLD*tan(angleMax);
	const float BRANCH_MIN_Y = BRANCH_DIFF_X_MIN*tan(angleMax);
	//const float BRANCH_HIGH_Y = BRANCH_RANGE_LIMIT*tan(angleMax);
	const float BRANCH_MAX_Y = BRANCH_DIST_LIMIT*tan(angleMax);

	//std::vector<float> listX;//goal_xを保存
	//std::vector<float> listY;//goal_yを保存
	std::vector<geometry_msgs::Point> list;
	geometry_msgs::Point tempGoal;


	//goal_x = 0;
	//goal_y = 0;


    //はじめのfor文では条件を満たした分岐領域を探す
    //２つ目のfor文で最も近い分岐を選ぶ
    //選んだ最も近い分岐に対して重複探査の確認を行いtrueが帰ってきたらその分岐を除いて再度最も近い分岐を探す

	for(int i=0;i<ranges.size()-1;i++){
		scanX = ranges[i]*cos(angles[i]);
		nextScanX = ranges[i+1]*cos(angles[i+1]);
		//if(scanX <= BRANCH_RANGE_LIMIT && nextScanX <= BRANCH_RANGE_LIMIT){
		if(scanX <= BRANCH_DIST_LIMIT && nextScanX <= BRANCH_DIST_LIMIT){
			diffX = std::abs(nextScanX - scanX);
			//if(diffX >= BRANCH_DIFF_THRESHOLD){
			if(diffX >= BRANCH_DIFF_X_MIN){
				scanY = ranges[i]*sin(angles[i]);
				nextScanY = ranges[i+1]*sin(angles[i+1]);
				diffY = std::abs(nextScanY - scanY);
				if(BRANCH_MIN_Y <= diffY && diffY <= BRANCH_MAX_Y){
					tempGoal.x = (nextScanX + scanX)/2;
					tempGoal.y = (nextScanY + scanY)/2;
					//listX.push_back((nextScanX + scanX)/2);
					//listY.push_back((nextScanY + scanY)/2);
					list.push_back(tempGoal);
					//flag = true;
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
		double yaw = 2*asin(pose.pose.orientation.z);

		std::vector<geometry_msgs::Point> globalList;
		globalList.resize(list.size());
		geometry_msgs::Point tempGlobal;

		for(int i=0;i<list.size();i++){
			tempGlobal.x = pose.pose.position.x + (cos(yaw)*list[i].x) - (sin(yaw)*list[i].y);
			tempGlobal.y = pose.pose.position.y + (cos(yaw)*list[i].y) + (sin(yaw)*list[i].y);
			globalList[i] = tempGlobal;
		}

		publishGoalList(globalList);

		for(int k=list.size();k>0;k--){
			centerDist = DOUBLE_INFINITY;
			for(int j=0;j<list.size();j++){
				tempCenterDist = std::abs(list[j].x)+std::abs(list[j].y);
				if(tempCenterDist <= centerDist){
					centerDist = tempCenterDist;
					//globalX = pose.pose.position.x+(cos(yaw)*goal.x) - (sin(yaw)*goal.y);
					//globalY = pose.pose.position.y+(cos(yaw)*goal.y) + (sin(yaw)*goal.x);
					//goal.x = list[j].x;
					//goal.y = list[j].y;
					//sensor->mapに座標変換
					//goal.x = pose.pose.position.x + (cos(yaw)*list[j].x) - (sin(yaw)*list[j].y);
					//goal.y = pose.pose.position.y + (cos(yaw)*list[j].y) + (sin(yaw)*list[j].y);
					goal = globalList[j];
					near = j;
				}
			}

			ROS_DEBUG_STREAM("Branch Candidate : (" << goal.x << "," << goal.y << ")\n");

			if(DUPLICATE_CHECK && duplicateDetection(goal)){
				list.erase(list.begin() + near);
				//listY.erase(list.begin() + near);
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

bool BranchSearch::duplicateDetection(geometry_msgs::Point goal){
	double globalX;//分岐領域の世界座標
	double globalY;//分岐領域の世界座標
	double xPlus;
	double xMinus;
	double yPlus;
	double yMinus;

	const int lOG_NEWER_LIMIT = 30;//ログの取得が1Hzの場合30秒前までのログで重複検出
	//bool duplication_flag = false;

	//odom_queue.callOne(ros::WallDuration(1));

	//poseLogData にポーズのログ
	qPoseLog.callOne(ros::WallDuration(1));

	//double yaw = 2*asin(pose.pose.orientation.z);


	//globalX = pose.pose.position.x+(cos(yaw)*goal.x) - (sin(yaw)*goal.y);
	//globalY = pose.pose.position.y+(cos(yaw)*goal.y) + (sin(yaw)*goal.x);

	xPlus = goal.x + DUPLICATE_MARGIN;
	xMinus = goal.x - DUPLICATE_MARGIN;
	yPlus = goal.y + DUPLICATE_MARGIN;
	yMinus = goal.y - DUPLICATE_MARGIN;

	//xPlus = globalX + DUPLICATE_MARGIN;
	//xMinus = globalX - DUPLICATE_MARGIN;
	//yPlus = globalY + DUPLICATE_MARGIN;
	//yMinus = globalY - DUPLICATE_MARGIN;


	//std::cout << "odom_x,odom_y (" << odom_x << "," << odom_y << ")" << std::endl;
	//std::cout << "goal_x,goal_y (" << goal_x << "," << goal_y << ")" << std::endl;
	//std::cout << "yaw (" << yaw << ")" << std::endl;
	//std::cout << "global_x,global_y (" << global_x << "," << global_y << ")" << std::endl;
	
	

	// for(int i=0;i<poseLogData.size()-lOG_NEWER_LIMIT;i++){
	// 	//過去のオドメトリが許容範囲の中に入っているか//
	// 	if((xPlus >= poseLogData[i].pose.position.x) && (xMinus <= poseLogData[i].pose.position.x)){
	// 		if((yPlus >= poseLogData[i].pose.position.y) && (yMinus <= poseLogData[i].pose.position.y)){
	// 			//duplication_flag = true;
	// 			//branch_find_flag = false;
	// 			//std::cout << "すでに探査した領域でした・・・ぐすん;;" << std::endl;
	// 			return true;
	// 		}
	// 	}
	// }

	for(int i=0;i<poseLogData.poses.size()-lOG_NEWER_LIMIT;i++){
		//過去のオドメトリが許容範囲の中に入っているか//
		if((xPlus >= poseLogData.poses[i].position.x) && (xMinus <= poseLogData.poses[i].position.x)){
			if((yPlus >= poseLogData.poses[i].position.y) && (yMinus <= poseLogData.poses[i].position.y)){
				//duplication_flag = true;
				//branch_find_flag = false;
				//std::cout << "すでに探査した領域でした・・・ぐすん;;" << std::endl;
				ROS_DEBUG_STREAM("This Branch is Duplicated\n");
				return true;
			}
		}
	}
	ROS_DEBUG_STREAM("This Branch is Not Duplicated\n");
	return false;
}

void BranchSearch::publishGoal(geometry_msgs::Point goal){
	geometry_msgs::PointStamped msg;
	msg.point = goal;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = mapFrameId;

	pubGoal.publish(msg);
	ROS_INFO_STREAM("Publish Goal\n");
}

void BranchSearch::publishGoalList(std::vector<geometry_msgs::Point> list){
	exploration_msgs::PointArray msg;
	msg.points = list;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = mapFrameId;

	pubGoalList.publish(msg);
	ROS_INFO_STREAM("Publish GoalList\n");
}

#endif //BRANCH_SEARCH_H
