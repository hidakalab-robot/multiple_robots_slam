#include <exploration/sensor_based_exploration.h>
#include <exploration_libraly/enum.h>
#include <exploration_libraly/convert.h>
#include <Eigen/Geometry>
#include <fstream>
#include <exploration_msgs/PointArray.h>
#include <exploration_msgs/PoseStampedArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <exploration/sensor_based_exploration_parameter_reconfigureConfig.h>
#include <exploration_libraly/struct.h>

namespace ExStc = ExpLib::Struct;
namespace ExCov = ExpLib::Convert;
namespace ExEnm = ExpLib::Enum;

SensorBasedExploration::SensorBasedExploration()
    :branch_(new ExStc::subStruct<exploration_msgs::PointArray>("branch", 1))
    ,pose_(new ExStc::subStruct<geometry_msgs::PoseStamped>("pose", 1))
    ,poseLog_(new ExStc::subStruct<exploration_msgs::PoseStampedArray>("pose_log", 1))
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

    std::vector<ExStc::listStruct> ls;
    ls.reserve(branch_->data.points.size());

    // 別形式 // 直前の目標と近い分岐は無視したい
    for(const auto& b : branch_->data.points){
        if(Eigen::Vector2d(b.x - lastGoal_->x, b.y - lastGoal_->y).norm()>LAST_GOAL_TOLERANCE) ls.emplace_back(b);
    }

    if(ls.size() == 0){
        ROS_ERROR_STREAM("Branch array became empty");
        return false;
    }

    // 重複探査検出
    duplicateDetection(ls, poseLog_->data);

    // goal を決定 // 適切なゴールが無ければ false
    return decideGoal(goal, ls, pose_->data);
}


void SensorBasedExploration::duplicateDetection(std::vector<ExStc::listStruct>& ls, const exploration_msgs::PoseStampedArray& log){
	//重複探査の新しさとかはヘッダーの時間で見る
	//重複が新しいときと古い時で挙動を変える
	
	//重複探査を考慮する時間の上限から参照する配列の最大値を設定
	int ARRAY_MAX = log.poses.size();
	for(int i=log.poses.size()-2;i!=0;--i){
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

void SensorBasedExploration::loadParams(void){
    ros::NodeHandle nh("~/sensor_based_exploration");
    // dynamic parameters
    nh.param<double>("last_goal_tolerance", LAST_GOAL_TOLERANCE, 1.0);
    nh.param<double>("duplicate_tolerance", DUPLICATE_TOLERANCE, 1.5);
    nh.param<double>("log_current_time", LOG_CURRENT_TIME, 10);
    nh.param<double>("newer_duplication_threshold", NEWER_DUPLICATION_THRESHOLD, 100);
    // static parameters
    nh.param<std::string>("sbe_parameter_file_path",SBE_PARAMETER_FILE_PATH,"sbe_last_parameters.yaml");
    nh.param<bool>("output_sbe_parameters",OUTPUT_SBE_PARAMETERS,true);
}

void SensorBasedExploration::dynamicParamsCB(exploration::sensor_based_exploration_parameter_reconfigureConfig &cfg, uint32_t level){
    LAST_GOAL_TOLERANCE = cfg.last_goal_tolerance;
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
    
    ofs << "last_goal_tolerance: " << LAST_GOAL_TOLERANCE << std::endl;
    ofs << "log_current_time: " << LOG_CURRENT_TIME << std::endl;
    ofs << "duplicate_tolerance: " << DUPLICATE_TOLERANCE << std::endl;
    ofs << "newer_duplication_threshold: " << NEWER_DUPLICATION_THRESHOLD << std::endl;
 }