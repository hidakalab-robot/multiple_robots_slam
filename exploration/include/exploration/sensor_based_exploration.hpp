#ifndef SENSOR_BASED_EXPLORATION_HPP
#define SENSOR_BASED_EXPLORATION_HPP

#include <geometry_msgs/PointStamped.h>
#include <exploration_msgs/PointArray.h>
#include <exploration_msgs/PoseStampedArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <exploration_libraly/struct.hpp>

class SensorBasedExploration
{
private:
    double LAST_GOAL_TOLERANCE;
    double LOG_NEWER_LIMIT;//if 30 -> 30秒前までのログで重複検出
    double DUPLICATE_TOLERANCE;
    double NEWER_DUPLICATION_THRESHOLD;//最近通った場所の重複とみなす時間の上限,時間の仕様はLOG_NEWER_LIMITと同じ


    geometry_msgs::Point lastGoal;

    ExpLib::subStruct<exploration_msgs::PointArray> branch_;
    ExpLib::subStruct<geometry_msgs::PoseStamped> pose_;
    ExpLib::subStruct<exploration_msgs::PoseStampedArray> log_;

    ExpLib::pubStruct<geometry_msgs::PointStamped> goal_;

    void duplicateDetection(std::vector<ExpLib::listStruct>& ls, const exploration_msgs::PoseStampedArray& log);
    bool decideGoal(geometry_msgs::PointStamped& goal, const std::vector<ExpLib::listStruct>& ls, const geometry_msgs::PoseStamped& pose);
    void publishGoal(const geometry_msgs::PointStamped& goal);
public:
    SensorBasedExploration();
    bool getGoal(geometry_msgs::PointStamped& goal);
};

SensorBasedExploration::SensorBasedExploration()
    :branch_("branch", 1)
    ,pose_("pose", 1)
    ,log_("log", 1)
    ,goal_("goal", 1){

    ros::NodeHandle p("~");
    p.param<double>("last_goal_tolerance", LAST_GOAL_TOLERANCE, 1.0);
    p.param<double>("log_newer_limit", LOG_NEWER_LIMIT, 10);
    p.param<double>("duplicate_tolerance", DUPLICATE_TOLERANCE, 1.5);
    p.param<double>("newer_duplication_threshold", NEWER_DUPLICATION_THRESHOLD, 100);
}

bool SensorBasedExploration::getGoal(geometry_msgs::PointStamped& goal){
    // 分岐の読み込み
    if(branch_.q.callOne(ros::WallDuration(1)) || branch_.data.points.size()==0){
        ROS_ERROR_STREAM("Can't read branch or don't find branch");  
        return false;
    }

    // pose の読みこみ
    if(pose_.q.callOne(ros::WallDuration(1))){
        ROS_ERROR_STREAM("Can't read pose");
        return false;
    }

    // log の読みこみ
    if(log_.q.callOne(ros::WallDuration(1))){
        ROS_ERROR_STREAM("Can't read pose log");
        return false;
    }

    std::vector<ExpLib::listStruct> ls;
    ls.reserve(branch_.data.points.size());

    // 別形式 // 直前の目標と近い分岐は無視したい
    for(const auto& b : branch_.data.points){
        if(Eigen::Vector2d(b.x - lastGoal.x, b.y - lastGoal.y).norm()<LAST_GOAL_TOLERANCE) ls.emplace_back(b);
    }

    if(ls.size() == 0) {
        ROS_ERROR_STREAM("Branch array became empty");
        return false;
    }

    // 重複探査検出
    duplicateDetection(ls, log_.data);

    // goal を決定 // 適切なゴールが無ければ false
    return decideGoal(goal, ls, pose_.data);
}


void SensorBasedExploration::duplicateDetection(std::vector<ExpLib::listStruct>& ls, const exploration_msgs::PoseStampedArray& log){
	//重複探査の新しさとかはヘッダーの時間で見る
	//重複が新しいときと古い時で挙動を変える
	
	//重複探査を考慮する時間の上限から参照する配列の最大値を設定
	int ARRAY_MAX = log.poses.size();
	for(int i=log.poses.size()-2;i!=0;--i){
		if(ros::Duration(log.header.stamp - log.poses[i].header.stamp).toSec() > LOG_NEWER_LIMIT){
			ARRAY_MAX = i;
			break;
		}
	}

    for(auto&& l : ls){
        for(int i=ARRAY_MAX;i!=0;--i){
            //過去のオドメトリが重複判定の範囲内に入っているか//
            if(Eigen::Vector2d(l.point.x - log.poses[i].pose.position.x, l.point.y - log.poses[i].pose.position.y).norm() < DUPLICATE_TOLERANCE){
                ROS_DEBUG_STREAM("This Branch is Duplicated");
                l.duplication = ros::Duration(log.header.stamp - log.poses[i].header.stamp).toSec() > NEWER_DUPLICATION_THRESHOLD ? ExpLib::DuplicationStatus::OLDER : ExpLib::DuplicationStatus::NEWER;
                break;
            }
        }
	}
}

bool SensorBasedExploration::decideGoal(geometry_msgs::PointStamped& goal, const std::vector<ExpLib::listStruct>& ls, const geometry_msgs::PoseStamped& pose){
    // 重複していない中で距離が一番近いやつ
    double dist = DBL_MAX;

    for(const auto& l : ls){
        if(l.duplication != ExpLib::DuplicationStatus::NOT_DUPLECATION) continue;
        double temp = Eigen::Vector2d(l.point.x - pose.pose.position.x, l.point.y - pose.pose.position.y).norm();
        if(temp <= dist){
            dist = std::move(temp);
			goal.point = l.point;
        }
    }

    if(dist < DBL_MAX){
        goal.header.frame_id = pose.header.frame_id;
        goal.header.stamp = ros::Time::now();
        lastGoal = goal.point;
        this->goal_.pub.publish(goal);
        return true;
    }
    return false;
}

#endif // SENSOR_BASED_EXPLORATION_HPP