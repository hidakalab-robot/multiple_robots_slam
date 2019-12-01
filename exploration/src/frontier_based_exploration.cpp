#include <exploration/frontier_based_exploration.h>
#include <fstream>
#include <Eigen/Geometry>
#include <exploration_libraly/convert.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <exploration_libraly/struct.h>
#include <exploration_msgs/FrontierArray.h>
#include <exploration/frontier_based_exploration_parameter_reconfigureConfig.h>
#include <dynamic_reconfigure/server.h>

namespace ExCov = ExpLib::Convert;
namespace ExStc = ExpLib::Struct;

FrontierBasedExploration::FrontierBasedExploration()
    :frontier_(new ExStc::subStruct<exploration_msgs::FrontierArray>("frontier", 1))
    ,pose_(new ExStc::subStruct<geometry_msgs::PoseStamped>("pose", 1))
    ,goal_(new ExStc::pubStruct<geometry_msgs::PointStamped>("goal", 1, true))
    ,drs_(new dynamic_reconfigure::Server<exploration::frontier_based_exploration_parameter_reconfigureConfig>(ros::NodeHandle("~/frontier_based_exploration")))
    ,lastGoal_(new geometry_msgs::Point()){
    loadParams();
    drs_->setCallback(boost::bind(&FrontierBasedExploration::dynamicParamsCB,this, _1, _2));
}

FrontierBasedExploration::~FrontierBasedExploration(){
    if(OUTPUT_FBE_PARAMETERS) outputParams();
}

bool FrontierBasedExploration::getGoal(geometry_msgs::PointStamped& goal){
    // 分岐の読み込み
    if(frontier_->q.callOne(ros::WallDuration(1)) || frontier_->data.frontiers.size()==0){
        ROS_ERROR_STREAM("Can't read frontier or don't find frontier");  
        return false;
    }
    // pose の読みこみ
    if(pose_->q.callOne(ros::WallDuration(1))){
        ROS_ERROR_STREAM("Can't read pose");
        return false;
    }

    std::vector<exploration_msgs::Frontier> frontiers(frontier_->data.frontiers);

    // frontier を条件でフィルタする
    if(LAST_GOAL_EFFECT){
        auto removeResult = std::remove_if(frontiers.begin(),frontiers.end(),[this](exploration_msgs::Frontier& f){return Eigen::Vector2d(f.point.x - lastGoal_->x, f.point.y - lastGoal_->y).norm()<LAST_GOAL_TOLERANCE;});
		frontiers.erase(std::move(removeResult),frontiers.end());
    }
    
    if(frontiers.size() == 0){
        ROS_ERROR_STREAM("Frontier array became empty");
        return false;
    }

    return decideGoal(goal, frontiers, pose_->data);
}

bool FrontierBasedExploration::decideGoal(geometry_msgs::PointStamped& goal, const std::vector<exploration_msgs::Frontier>& frontiers,const geometry_msgs::PoseStamped& pose){
    //現在位置からそれぞれのフロンティア座標に対して距離とベクトルを計算し、評価関数によって目標を決定

    //ロボットの向きのベクトル(大きさ1)を計算
    Eigen::Vector2d directionVec = ExCov::qToVector2d(pose.pose.orientation);

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
        *lastGoal_ = goal.point;
        goal_->pub.publish(goal);
        return true;
    }
    return false;
}

void FrontierBasedExploration::loadParams(void){
    ros::NodeHandle nh("~/frontier_based_exploration");
    // dynamic parameters
    nh.param<double>("distance_weight", DISTANCE_WEIGHT, 1.0);
    nh.param<double>("direction_weight", DIRECTION_WEIGHT, 0.0);
    nh.param<bool>("last_goal_effect", LAST_GOAL_EFFECT, true);
    nh.param<double>("last_goal_tolerance", LAST_GOAL_TOLERANCE, 0.5);
    // static parameters
    nh.param<std::string>("fbe_parameter_file_path",FBE_PARAMETER_FILE_PATH,"fbe_last_parameters.yaml");
    nh.param<bool>("output_fbe_parameters",OUTPUT_FBE_PARAMETERS,true);
}

void FrontierBasedExploration::dynamicParamsCB(exploration::frontier_based_exploration_parameter_reconfigureConfig &cfg, uint32_t level){
    DISTANCE_WEIGHT = cfg.distance_weight;
    DIRECTION_WEIGHT = cfg.direction_weight;
    LAST_GOAL_EFFECT = cfg.last_goal_effect;
    LAST_GOAL_TOLERANCE = cfg.last_goal_tolerance;
}

void FrontierBasedExploration::outputParams(void){
    std::cout << "writing fbe last parameters ... ..." << std::endl;
    std::ofstream ofs(FBE_PARAMETER_FILE_PATH);

    if(ofs) std::cout << "fbe param file open succeeded" << std::endl;
    else {
        std::cout << "fbe param file open failed" << std::endl;
        return;
    }

    ofs << "distance_weight: " << DISTANCE_WEIGHT << std::endl;
    ofs << "direction_weight: " << DIRECTION_WEIGHT << std::endl;
    ofs << "last_goal_effect: " << (LAST_GOAL_EFFECT ? "true" : "false") << std::endl;
    ofs << "last_goal_tolerance: " << LAST_GOAL_TOLERANCE << std::endl;
 }