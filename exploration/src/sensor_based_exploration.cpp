#include <exploration/sensor_based_exploration.h>
#include <exploration_libraly/enum.h>
#include <exploration_libraly/convert.h>
#include <exploration_libraly/utility.h>
#include <Eigen/Geometry>
#include <fstream>
#include <exploration_msgs/PointArray.h>
// #include <exploration_msgs/PoseStampedArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <exploration/sensor_based_exploration_parameter_reconfigureConfig.h>
#include <exploration_libraly/struct.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

namespace ExStc = ExpLib::Struct;
namespace ExCov = ExpLib::Convert;
namespace ExEnm = ExpLib::Enum;
namespace ExUtl = ExpLib::Utility;

SensorBasedExploration::SensorBasedExploration()
    :branch_(new ExStc::subStruct<exploration_msgs::PointArray>("branch", 1))
    ,pose_(new ExStc::subStruct<geometry_msgs::PoseStamped>("pose", 1))
    // ,poseLog_(new ExStc::subStruct<exploration_msgs::PoseStampedArray>("pose_log", 1))
    ,poseLog_(new ExStc::subStruct<nav_msgs::Path>("pose_log", 1))
    ,canceled_(new ExStc::subStruct<exploration_msgs::PointArray>("canceled_goals", 1))
    ,map_(new ExStc::subStruct<nav_msgs::OccupancyGrid>("map", 1))
    ,dupBra_(new ExStc::pubStruct<exploration_msgs::PointArray>("duplicated_branch", 1, true))
    ,onMapBra_(new ExStc::pubStruct<exploration_msgs::PointArray>("on_map_branch", 1, true))
    ,goal_(new ExStc::pubStruct<geometry_msgs::PointStamped>("goal", 1, true))
    ,drs_(new dynamic_reconfigure::Server<exploration::sensor_based_exploration_parameter_reconfigureConfig>(ros::NodeHandle("~/sensor_based_exploration")))
    ,lastGoal_(new geometry_msgs::Point()){
    loadParams();
    drs_->setCallback(boost::bind(&SensorBasedExploration::dynamicParamsCB,this, _1, _2));
}

SensorBasedExploration::~SensorBasedExploration(){
    if(OUTPUT_SBE_PARAMETERS) outputParams();
}

bool SensorBasedExploration::getGoal(geometry_msgs::PointStamped& goal){
    // 分岐の読み込み
    if(branch_->q.callOne(ros::WallDuration(1)) || branch_->data.points.size()==0){
        ROS_ERROR_STREAM("Can't read branch or don't find branch");  
        return false;
    }

    // pose の読みこみ
    if(pose_->q.callOne(ros::WallDuration(1))){
        ROS_ERROR_STREAM("Can't read pose");
        return false;
    }

    // log の読みこみ
    if(poseLog_->q.callOne(ros::WallDuration(1))){
        ROS_ERROR_STREAM("Can't read pose log");
        return false;
    }

    std::vector<geometry_msgs::Point> branches(branch_->data.points);

    //　最後のゴールと近かったら削除
    if(LAST_GOAL_EFFECT){
        auto removeResult = std::remove_if(branches.begin(),branches.end(),[this](geometry_msgs::Point& p){return Eigen::Vector2d(p.x - lastGoal_->x, p.y - lastGoal_->y).norm()<LAST_GOAL_TOLERANCE;});
		branches.erase(std::move(removeResult),branches.end());
    }

    if(CANCELED_GOAL_EFFECT && !canceled_->q.callOne(ros::WallDuration(1)) && canceled_->data.points.size()!=0){
        auto removeResult = std::remove_if(branches.begin(),branches.end(),[this](geometry_msgs::Point& p){
            for(const auto& c : canceled_->data.points){
                if(Eigen::Vector2d(p.x - c.x, p.y - c.y).norm()<CANCELED_GOAL_TOLERANCE) return true;
            }
            return false;
        });
		branches.erase(std::move(removeResult),branches.end());
    }

    if(branches.size() == 0){
        ROS_ERROR_STREAM("Branch array became empty");
        return false;
    }

    std::vector<ExStc::listStruct> ls;
    ls.reserve(branch_->data.points.size());
    for(const auto& b : branches) ls.emplace_back(b);

    // 重複探査検出
    if(DUPLICATE_DETECTION) duplicateDetection(ls, poseLog_->data);

    // 行ったことがなくても地図ができてたら重複探査にする
    if(ON_MAP_BRANCH_DETECTION && !map_->q.callOne(ros::WallDuration(1))) onMapBranchDetection(ls);

    publishProcessedBranch(ls);

    // for(auto&& l : ls) ROS_DEBUG_STREAM("\n branch : (" << l.point.x << ", " << l.point.y << ") , status : " << (int)l.duplication << "\n");

    // goal を決定 // 適切なゴールが無ければ false
    return decideGoal(goal, ls, pose_->data);
}

// void SensorBasedExploration::duplicateDetection(std::vector<ExStc::listStruct>& ls, const exploration_msgs::PoseStampedArray& log){
void SensorBasedExploration::duplicateDetection(std::vector<ExStc::listStruct>& ls, const nav_msgs::Path& log){
	//重複探査の新しさとかはヘッダーの時間で見る
	//重複が新しいときと古い時で挙動を変える
	
	//重複探査を考慮する時間の上限から参照する配列の最大値を設定
	int ARRAY_MAX = log.poses.size()-1;
	for(int i=log.poses.size()-1;i!=0;--i){
		if(ros::Duration(log.header.stamp - log.poses[i].header.stamp).toSec() > LOG_CURRENT_TIME){
			ARRAY_MAX = i;
			break;
		}
	}
    for(auto&& l : ls){
        for(int i=ARRAY_MAX;i!=0;--i){
            //過去のオドメトリが重複判定の範囲内に入っているか//
            if(Eigen::Vector2d(l.point.x - log.poses[i].pose.position.x, l.point.y - log.poses[i].pose.position.y).norm() < DUPLICATE_TOLERANCE){
                ROS_DEBUG_STREAM("This Branch is Duplicated");
                l.duplication = ros::Duration(log.header.stamp - log.poses[i].header.stamp).toSec() > NEWER_DUPLICATION_THRESHOLD ? ExEnm::DuplicationStatus::OLDER : ExEnm::DuplicationStatus::NEWER;
                break;
            }
        }
	}
}

void SensorBasedExploration::onMapBranchDetection(std::vector<ExStc::listStruct>& ls){
    // 分岐があり行ったことがない場所でも既に地図ができているところを検出する
    // パラメータで検索窓を作ってその窓の中で地図ができている割合が一定以上であれば地図ができているという判定にする
    std::vector<std::vector<int8_t>> map2d = ExUtl::mapArray1dTo2d(map_->data.data,map_->data.info);
    for(auto&& l : ls){
        if(l.duplication != ExEnm::DuplicationStatus::NOT_DUPLECATION) continue;
        ExStc::mapSearchWindow msw(l.point,map_->data.info,OMB_MAP_WINDOW_X,OMB_MAP_WINDOW_Y);
        int c = 0;
        for(int y=msw.top,ey=msw.bottom+1;y!=ey;++y){
            for(int x=msw.left,ex=msw.right+1;x!=ex;++x){
                if(map2d[x][y] >= 0) ++c;
            }
        }
        if((double)c/(msw.width*msw.height)>ON_MAP_BRANCH_RATE) l.duplication = ExEnm::DuplicationStatus::ON_MAP;
    }
}

bool SensorBasedExploration::decideGoal(geometry_msgs::PointStamped& goal, const std::vector<ExStc::listStruct>& ls, const geometry_msgs::PoseStamped& pose){
    // 重複していない中で距離が一番近いやつ
    double dist = DBL_MAX;

    for(const auto& l : ls){
        if(l.duplication != ExEnm::DuplicationStatus::NOT_DUPLECATION) continue;
        double temp = Eigen::Vector2d(l.point.x - pose.pose.position.x, l.point.y - pose.pose.position.y).norm();
        if(temp <= dist){
            dist = std::move(temp);
			goal.point = l.point;
        }
    }

    if(dist < DBL_MAX){
        goal.header.frame_id = pose.header.frame_id;
        goal.header.stamp = ros::Time::now();
        *lastGoal_ = goal.point;
        goal_->pub.publish(goal);
        return true;
    }
    return false;
}

void SensorBasedExploration::publishProcessedBranch(const std::vector<ExStc::listStruct>& ls){
    exploration_msgs::PointArray dup,om;
    dup.points.reserve(ls.size());
    om.points.reserve(ls.size());
    for(auto&& l : ls){
        switch (l.duplication){
            case ExEnm::DuplicationStatus::NEWER:
                dup.points.emplace_back(l.point);
                break;
            case ExEnm::DuplicationStatus::ON_MAP:
                om.points.emplace_back(l.point);
                break;
            default:
                break;
        }
    }
    dup.header.frame_id = om.header.frame_id = branch_->data.header.frame_id;
    dup.header.stamp = om.header.stamp = ros::Time::now();
    dupBra_->pub.publish(dup);
    onMapBra_->pub.publish(om);
}

void SensorBasedExploration::loadParams(void){
    ros::NodeHandle nh("~/sensor_based_exploration");
    // dynamic parameters
    nh.param<bool>("last_goal_effect", LAST_GOAL_EFFECT, true);
    nh.param<double>("last_goal_tolerance", LAST_GOAL_TOLERANCE, 1.0);
    nh.param<bool>("canceled_goal_effect", CANCELED_GOAL_EFFECT, true);
    nh.param<double>("canceled_goal_tolerance", CANCELED_GOAL_TOLERANCE, 0.5);
    nh.param<bool>("on_map_branch_detection", ON_MAP_BRANCH_DETECTION, true);
    nh.param<double>("on_map_branch_rate", ON_MAP_BRANCH_RATE, 0.5);
    nh.param<double>("omb_map_window_x", OMB_MAP_WINDOW_X, 1.0);
    nh.param<double>("omb_map_window_y", OMB_MAP_WINDOW_Y, 1.0);
    nh.param<bool>("duplicate_detection", DUPLICATE_DETECTION, true);
    nh.param<double>("duplicate_tolerance", DUPLICATE_TOLERANCE, 1.5);
    nh.param<double>("log_current_time", LOG_CURRENT_TIME, 10);
    nh.param<double>("newer_duplication_threshold", NEWER_DUPLICATION_THRESHOLD, 100);
    // static parameters
    nh.param<std::string>("sbe_parameter_file_path",SBE_PARAMETER_FILE_PATH,"sbe_last_parameters.yaml");
    nh.param<bool>("output_sbe_parameters",OUTPUT_SBE_PARAMETERS,true);
}

void SensorBasedExploration::dynamicParamsCB(exploration::sensor_based_exploration_parameter_reconfigureConfig &cfg, uint32_t level){
    LAST_GOAL_EFFECT = cfg.last_goal_effect;
    LAST_GOAL_TOLERANCE = cfg.last_goal_tolerance;
    CANCELED_GOAL_EFFECT = cfg.canceled_goal_effect;
    CANCELED_GOAL_TOLERANCE = cfg.canceled_goal_tolerance;
    ON_MAP_BRANCH_DETECTION = cfg.on_map_branch_detection;
    ON_MAP_BRANCH_RATE = cfg.on_map_branch_rate;
    OMB_MAP_WINDOW_X = cfg.omb_map_window_x;
    OMB_MAP_WINDOW_Y = cfg.omb_map_window_y;
    DUPLICATE_DETECTION = cfg.duplicate_detection;
    DUPLICATE_TOLERANCE = cfg.duplicate_tolerance;
    LOG_CURRENT_TIME = cfg.log_current_time;
    NEWER_DUPLICATION_THRESHOLD = cfg.newer_duplication_threshold;
}

void SensorBasedExploration::outputParams(void){
    std::cout << "writing sbe last parameters ... ..." << std::endl;
    std::ofstream ofs(SBE_PARAMETER_FILE_PATH);

    if(ofs) std::cout << "sbe param file open succeeded" << std::endl;
    else {
        std::cout << "sbe param file open failed" << std::endl;
        return;
    }
    
    ofs << "last_goal_effect: " << (LAST_GOAL_EFFECT ? "true" : "false") << std::endl;
    ofs << "last_goal_tolerance: " << LAST_GOAL_TOLERANCE << std::endl;
    ofs << "canceled_goal_effect: " << (CANCELED_GOAL_EFFECT ? "true" : "false") << std::endl;
    ofs << "canceled_goal_tolerance: " << CANCELED_GOAL_TOLERANCE << std::endl;
    ofs << "on_map_branch_detection: " << (ON_MAP_BRANCH_DETECTION ? "true" : "false") << std::endl;
    ofs << "on_map_branch_tolerance: " << ON_MAP_BRANCH_RATE << std::endl;
    ofs << "omb_map_window_x: " << OMB_MAP_WINDOW_X << std::endl;
    ofs << "omb_map_window_y: " << OMB_MAP_WINDOW_Y << std::endl;
    ofs << "duplicate_detection: " << (DUPLICATE_DETECTION ? "true" : "false") << std::endl;
    ofs << "duplicate_tolerance: " << DUPLICATE_TOLERANCE << std::endl;
    ofs << "log_current_time: " << LOG_CURRENT_TIME << std::endl;
    ofs << "newer_duplication_threshold: " << NEWER_DUPLICATION_THRESHOLD << std::endl;
 }