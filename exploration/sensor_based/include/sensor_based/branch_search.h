#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>

//分岐領域を検出
class BranchSearch
{
private:
    double BRANCH_ANGLE;
    double CENTER_RANGE_MIN;
    double BRANCH_RANGE_LIMIT;
    double BRANCH_DIFF_THRESHOLD;


public:
    BranchSearch();
    ~BranchSearch(){};
    void initialize(double branchAngle, double centerRangeMin);
    geometry_msgs::Point returnBranch(sensor_msgs::LaserScan scanData);
    bool branchDetection(std::vector<float>& ranges, std::vector<float>& angles,float angleMax,geometry_msgs::Point& goal);
    bool duplicatedPointDetection(void);
};

BranchSearch::BranchSearch(){

}

void BranchSearch::initialize(double branchAngle, double centerRangeMin){
    BRANCH_ANGLE = branchAngle;
    CENTER_RANGE_MIN = centerRangeMin;
    //BRANCH_RANGE_LIMIT = 
    //BRANCH_DIFF_THRESHOLD
}

geometry_msgs::Point BranchSearch::returnBranch(sensor_msgs::LaserScan scanData){
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
    if(!branchDetection(fixRanges,fixAngle,scanData.angle_max,goal)){
        geometry_msgs::Point temp;
        goal = temp
    }

    return goal;

}

bool BranchSearch::branchDetection(std::vector<float> ranges, std::vector<float> angles,float angleMax,geometry_msgs::Point& goal){
    //int i;
	//int j;
	int near;
	float scanX;
	float scanY;
	float nextScanX;
	float nextScanY;
	float diffX;
    float diffY;

	float centerDist = 1000.0;

	//float Branch_y_dist;

	//分岐のy座標の差がこの値の範囲内の場合のみ分岐として検出
	const float BRANCH_LOW_Y = BRANCH_DIFF_THRESHOLD*tan(angleMax);
	const float BRANCH_HIGH_Y = BRANCH_RANGE_LIMIT*tan(angleMax);//分岐領域の2点間のｙ座標の差がこの値以下のとき分岐として検出

	std::vector<float> listX;//goal_xを保存
	std::vector<float> listY;//goal_yを保存

	//goal_x = 0;
	//goal_y = 0;

    bool flag;

    //はじめのfor文では条件を満たした分岐領域を探す
    //２つ目のfor文で最も近い分岐を選ぶ
    //選んだ最も近い分岐に対して重複探査の確認を行いtrueが帰ってきたらその分岐を除いて再度最も近い分岐を探す

	for(int i=0;i<ranges.size()-1;i++){
		scanX = ranges[i]*cos(angles[i]);
		nextScanX = ranges[i+1]*cos(angles[i+1]);
		if(scanX <= BRANCH_RANGE_LIMIT && nextScanX <= BRANCH_RANGE_LIMIT){
			diffX = std::abs(nextScanX - scanX);
			if(diffX >= BRANCH_DIFF_THRESHOLD){
				scanY = ranges[i]*sin(angles[i]);
				nextScanY = ranges[i+1]*sin(angles[i+1]);
				diffY = std::abs(nextScanY - scanY);
				if(BRANCH_LOW_Y <= diffY && diffY <= BRANCH_HIGH_Y){
					listX.push_back((nextScanX + scanX)/2);
					listY.push_back((nextScanY + scanY)/2);
					flag = true;
				}
			}
		}
	}

	if(flag){
        bool tempCenterDist;
		for(int k=listX.size();k>0;k--){
			for(int j=0;j<listX.size();j++){
				tempCenterDist = std::abs(listX[j])+std::abs(listY[j]);
				if(tempCenterDist <= centerDist){
					centerDist = tempCenterDist;
					goal.x = listX[j];
					goal.y = listY[j];
					near = j;
				}
			}
			if(duplicatedDetection()){
				listX.erase(listX.begin() + near);
				listY.erase(listY.begin() + near);
				flag = false;
			}
			else{
				flag = true;
				break;
			}
			centerDist = 1000.0;
		}	
    }

    return flag;
}

void BranchSearch::duplicatedDetection(void){

}
