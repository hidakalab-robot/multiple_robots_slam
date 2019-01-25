#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
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
	std::string ROBOT_NAME;

	ros::NodeHandle spl;
    ros::Subscriber subPoseLog;
    ros::CallbackQueue qPoseLog;
    std::string poseLogTopic;
    //std::vector<geometry_msgs::PoseStamped> poseLogData;
	geometry_msgs::PoseArray poseLogData;

	ros::NodeHandle pgb;
	ros::Publisher pubGoalBranch;
	std::string goalBranchTopic;


public:
    BranchSearch();
    ~BranchSearch(){};
    //void initialize(double branchAngle, double centerRangeMin);
    geometry_msgs::Point getGoalBranch(sensor_msgs::LaserScan scanData,geometry_msgs::PoseStamped pose);
    bool branchDetection(std::vector<float>& ranges, std::vector<float>& angles,float angleMax,geometry_msgs::Point& goal,geometry_msgs::PoseStamped pose);
    bool duplicateDetection(geometry_msgs::Point goal);

	//void poseLogCB(const sensor_based_exploration::PoseLog::ConstPtr& msg);
	void poseLogCB(const geometry_msgs::PoseArray::ConstPtr& msg);

	void publishGoalBranch(geometry_msgs::Point goal);
//branchListをパブリッシュした方が色々使えるかも
};

BranchSearch::BranchSearch(){
	p.getParam("pose_log_topic", poseLogTopic);
    spl.setCallbackQueue(&qPoseLog);
    subPoseLog = spl.subscribe(poseLogTopic,1,&BranchSearch::poseLogCB, this);

	pubGoalBranch = pgb.advertise<geometry_msgs::PointStamped>(goalBranchTopic, 1);

	//branch_searchパラメータの読み込み
	p.param<double>("branch_angle", BRANCH_ANGLE, 1.0);
	p.param<double>("center_range_min", CENTER_RANGE_MIN, 1.0);
	//p.param<double>("branch_range_limit", BRANCH_RANGE_LIMIT, 1.0);
	p.param<double>("branch_range_limit", BRANCH_DIST_LIMIT, 1.0);
	//p.param<double>("branch_diff_threshold", BRANCH_DIFF_THRESHOLD, 1.0);
	p.param<double>("branch_diff_x_min", BRANCH_DIFF_X_MIN, 1.0);
	p.param<std::string>("robot_name", ROBOT_NAME, "robot1");
	p.param<double>("duplicate_margin", DUPLICATE_MARGIN, 1.0);

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

void BranchSearch::publishGoalBranch(geometry_msgs::Point goal){
	geometry_msgs::PointStamped msg;
	msg.point = goal;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = ROBOT_NAME + "/map";

	pubGoalBranch.publish(msg);
}

geometry_msgs::Point BranchSearch::getGoalBranch(sensor_msgs::LaserScan scanData, geometry_msgs::PoseStamped pose){
    const int scanWidth = BRANCH_ANGLE / scanData.angle_increment;
    const int scanMin = (scanData.ranges.size()/2)-1 - scanWidth;
    const int scanMax = (scanData.ranges.size()/2) + scanWidth;

    geometry_msgs::Point goal;

    for(int i = scanMin;i<scanMax;i++){
		if(!std::isnan(scanData.ranges[i]) && scanData.ranges[i] < CENTER_RANGE_MIN){
			return goal;
		}
    }

    std::vector<float> fixRanges;
    std::vector<float> fixAngles;

    for(int i=0;i<scanData.ranges.size();i++){
		if(!std::isnan(scanData.ranges[i])){
			fixRanges.push_back(scanData.ranges[i]);
			fixAngles.push_back(scanData.angle_min+(scanData.angle_increment*i));
		}
    }

    if(fixRanges.size() < 2){
		return goal;
    }

    //bool branchFlag;

    //branchFlag = branchDetection(fixRanges,fixAngle,scanData.angle_max,goal);
    if(branchDetection(fixRanges,fixAngles,scanData.angle_max,goal,pose)){
		//double yaw = 2*asin(pose.pose.orientation.z);
		//goal.x = pose.pose.position.x+(cos(yaw)*goal.x) - (sin(yaw)*goal.y);
		//globalY = pose.pose.position.y+(cos(yaw)*goal.y) + (sin(yaw)*goal.x);
		publishGoalBranch(goal);
    }
	else{
		geometry_msgs::Point temp;
        goal = temp;
	}
	//publishGoalBranch(goal);

    return goal;

}

bool BranchSearch::branchDetection(std::vector<float>& ranges, std::vector<float>& angles,float angleMax,geometry_msgs::Point& goal,geometry_msgs::PoseStamped pose){

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
		int near;
		float centerDist;
        bool tempCenterDist;
		double yaw = 2*asin(pose.pose.orientation.z);

		for(int k=list.size();k>0;k--){
			centerDist = 1000.0;
			for(int j=0;j<list.size();j++){
				tempCenterDist = std::abs(list[j].x)+std::abs(list[j].y);
				if(tempCenterDist <= centerDist){
					centerDist = tempCenterDist;
					//globalX = pose.pose.position.x+(cos(yaw)*goal.x) - (sin(yaw)*goal.y);
					//globalY = pose.pose.position.y+(cos(yaw)*goal.y) + (sin(yaw)*goal.x);
					//goal.x = list[j].x;
					//goal.y = list[j].y;
					//sensor->mapに座標変換
					goal.x = pose.pose.position.x + (cos(yaw)*list[j].x) - (sin(yaw)*list[j].y);
					goal.y = pose.pose.position.y + (cos(yaw)*list[j].y) + (sin(yaw)*list[j].y);
					near = j;
				}
			}
			if(duplicateDetection(goal)){
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

	xPlus = globalX + DUPLICATE_MARGIN;
	xMinus = globalX - DUPLICATE_MARGIN;
	yPlus = globalY + DUPLICATE_MARGIN;
	yMinus = globalY - DUPLICATE_MARGIN;


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
				return true;
			}
		}
	}
	
	return false;
}
