#ifndef FRONTIER_BASED_EXPLORATION_HPP
#define FRONTIER_BASED_EXPLORATION_HPP

#include <exploration_libraly/convert.hpp>
#include <exploration_libraly/struct.hpp>
#include <exploration_msgs/FrontierArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

class FrontierBasedExploration
{
private:
    bool LAST_GOAL_EFFECT;
    double LAST_GOAL_TOLERANCE;
    double DISTANCE_WEIGHT;
    double DIRECTION_WEIGHT;

    geometry_msgs::Point lastGoal_;

    ExpLib::Struct::subStruct<exploration_msgs::FrontierArray> frontier_;
    ExpLib::Struct::subStruct<geometry_msgs::PoseStamped> pose_;
    ExpLib::Struct::pubStruct<geometry_msgs::PointStamped> goal_;

    void frontierFilter(std::vector<exploration_msgs::Frontier>& frontiers);
    bool decideGoal(geometry_msgs::PointStamped& goal, const std::vector<exploration_msgs::Frontier>& frontiers, const geometry_msgs::PoseStamped& pose);
public:
    FrontierBasedExploration();
    bool getGoal(geometry_msgs::PointStamped& goal);
};

FrontierBasedExploration::FrontierBasedExploration()
    :frontier_("frontier", 1)
    ,pose_("pose", 1)
    ,goal_("goal", 1){

    ros::NodeHandle p("~");
    p.param<bool>("last_goal_effect", LAST_GOAL_EFFECT, true);
    p.param<double>("last_goal_tolerance", LAST_GOAL_TOLERANCE, 0.5);
    p.param<double>("distance_weight", DISTANCE_WEIGHT, 1.0);
    p.param<double>("direction_weight", DIRECTION_WEIGHT, 0.0);
}

bool FrontierBasedExploration::getGoal(geometry_msgs::PointStamped& goal){
    // 分岐の読み込み
    if(frontier_.q.callOne(ros::WallDuration(1)) || frontier_.data.frontiers.size()==0){
        ROS_ERROR_STREAM("Can't read frontier or don't find frontier");  
        return false;
    }

    // pose の読みこみ
    if(pose_.q.callOne(ros::WallDuration(1))){
        ROS_ERROR_STREAM("Can't read pose");
        return false;
    }

    std::vector<exploration_msgs::Frontier> frontiers(frontier_.data.frontiers);

    frontierFilter(frontiers);
    
    if(frontiers.size() == 0){
        ROS_ERROR_STREAM("Frontier array became empty");
        return false;
    }

    return decideGoal(goal, frontiers, pose_.data);
}

void FrontierBasedExploration::frontierFilter(std::vector<exploration_msgs::Frontier>& frontiers){
    // frontier を条件でフィルタする
    if(LAST_GOAL_EFFECT){
        auto removeResult = std::remove_if(frontiers.begin(),frontiers.end(),[this](exploration_msgs::Frontier& f){return Eigen::Vector2d(f.point.x - lastGoal_.x, f.point.y - lastGoal_.y).norm()<LAST_GOAL_TOLERANCE;});
		frontiers.erase(std::move(removeResult),frontiers.end());
    }
}

bool FrontierBasedExploration::decideGoal(geometry_msgs::PointStamped& goal, const std::vector<exploration_msgs::Frontier>& frontiers,const geometry_msgs::PoseStamped& pose){
    //現在位置からそれぞれのフロンティア座標に対して距離とベクトルを計算し、評価関数によって目標を決定

    //ロボットの向きのベクトル(大きさ1)を計算
    Eigen::Vector2d directionVec = ExpLib::Convert::qToVector2d(pose.pose.orientation);

    double max = -DBL_MAX;
    for(auto& f : frontiers){
        Eigen::Vector2d v(f.point.x - pose.pose.position.x, f.point.y - pose.pose.position.y);
        //評価値が最大となる目標値を選択
        double value = DIRECTION_WEIGHT * v.normalized().dot(directionVec) - DISTANCE_WEIGHT * v.norm();
        if(value > max){
            max = std::move(value);
            goal.point = f.point;
        }
    }

    if(max > -DBL_MAX){
        goal.header.frame_id = pose.header.frame_id;
        goal.header.stamp = ros::Time::now();
        lastGoal_ = goal.point;
        goal_.pub.publish(goal);
        return true;
    }
    return false;
}

#endif // FRONTIER_BASED_EXPLORATION_HPP