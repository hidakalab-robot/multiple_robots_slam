#ifndef FRONTIER_BASED_EXPLORATION_HPP
#define FRONTIER_BASED_EXPLORATION_HPP

#include <exploration_libraly/convert.hpp>
#include <exploration_libraly/struct.hpp>
#include <exploration_msgs/FrontierArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <dynamic_reconfigure/server.h>
#include <exploration/frontier_based_exploration_parameter_reconfigureConfig.h>
#include <fstream>

namespace ExStc = ExpLib::Struct;
class FrontierBasedExploration
{
private:
    bool LAST_GOAL_EFFECT;
    double LAST_GOAL_TOLERANCE;
    double DISTANCE_WEIGHT;
    double DIRECTION_WEIGHT;

    geometry_msgs::Point lastGoal_;

    ExStc::subStruct<exploration_msgs::FrontierArray> frontier_;
    ExStc::subStruct<geometry_msgs::PoseStamped> pose_;
    ExStc::pubStruct<geometry_msgs::PointStamped> goal_;

    ros::NodeHandle nh;
    dynamic_reconfigure::Server<exploration::frontier_based_exploration_parameter_reconfigureConfig> server;
    dynamic_reconfigure::Server<exploration::frontier_based_exploration_parameter_reconfigureConfig>::CallbackType cbt;
    bool OUTPUT_FBE_PARAMETERS;
    std::string FBE_PARAMETER_FILE_PATH;

    void frontierFilter(std::vector<exploration_msgs::Frontier>& frontiers);
    bool decideGoal(geometry_msgs::PointStamped& goal, const std::vector<exploration_msgs::Frontier>& frontiers, const geometry_msgs::PoseStamped& pose);
    void dynamicParamCallback(exploration::frontier_based_exploration_parameter_reconfigureConfig &cfg, uint32_t level);
    void outputParams(void);
public:
    FrontierBasedExploration();
    ~FrontierBasedExploration(){if(OUTPUT_FBE_PARAMETERS) outputParams();};
    bool getGoal(geometry_msgs::PointStamped& goal);
};

FrontierBasedExploration::FrontierBasedExploration()
    :frontier_("frontier", 1)
    ,pose_("pose", 1)
    ,goal_("goal", 1, true)
    ,nh("~/frontier_based_exploration")
    ,server(nh){

    nh.param<bool>("last_goal_effect", LAST_GOAL_EFFECT, true);
    nh.param<double>("last_goal_tolerance", LAST_GOAL_TOLERANCE, 0.5);
    nh.param<double>("distance_weight", DISTANCE_WEIGHT, 1.0);
    nh.param<double>("direction_weight", DIRECTION_WEIGHT, 0.0);
    nh.param<bool>("output_fbe_parameters",OUTPUT_FBE_PARAMETERS,true);
    nh.param<std::string>("fbe_parameter_file_path",FBE_PARAMETER_FILE_PATH,"fbe_last_parameters.yaml");

    cbt = boost::bind(&FrontierBasedExploration::dynamicParamCallback,this, _1, _2);
    server.setCallback(cbt);
}

void FrontierBasedExploration::dynamicParamCallback(exploration::frontier_based_exploration_parameter_reconfigureConfig &cfg, uint32_t level){
    LAST_GOAL_EFFECT = cfg.last_goal_effect;
    LAST_GOAL_TOLERANCE = cfg.last_goal_tolerance;
    DISTANCE_WEIGHT = cfg.distance_weight;
    DIRECTION_WEIGHT = cfg.direction_weight;
}

void FrontierBasedExploration::outputParams(void){
    std::cout << "writing last parameters ... ..." << std::endl;
    std::ofstream ofs(FBE_PARAMETER_FILE_PATH);

    if(ofs) std::cout << "file open succeeded" << std::endl;
    else {
        std::cout << "file open failed" << std::endl;
        return;
    }
    ofs << "last_goal_effect: " << (LAST_GOAL_EFFECT ? "true" : "false") << std::endl;
    ofs << "last_goal_tolerance: " << LAST_GOAL_TOLERANCE << std::endl;
    ofs << "distance_weight: " << DISTANCE_WEIGHT << std::endl;
    ofs << "direction_weight: " << DIRECTION_WEIGHT << std::endl;
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