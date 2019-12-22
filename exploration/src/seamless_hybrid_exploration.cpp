#include <exploration/seamless_hybrid_exploration.h>
#include <exploration_libraly/convert.h>
#include <fstream>
#include <exploration_libraly/struct.h>
#include <exploration_libraly/enum.h>
#include <exploration_libraly/path_planning.h>
#include <exploration_libraly/utility.h>
#include <exploration_msgs/BranchArray.h>
#include <exploration_msgs/FrontierArray.h>
#include <exploration_msgs/PointArray.h>
#include <exploration_msgs/RobotInfoArray.h>
#include <navfn/navfn_ros.h>
#include <dynamic_reconfigure/server.h>
#include <exploration/seamless_hybrid_exploration_parameter_reconfigureConfig.h>
#include <exploration/sensor_based_exploration_parameter_reconfigureConfig.h>
#include <geometry_msgs/Point.h>

namespace ExStc = ExpLib::Struct;
namespace ExCov = ExpLib::Convert;
namespace ExEnm = ExpLib::Enum;
namespace ExUtl = ExpLib::Utility;

struct SeamlessHybridExploration::maxValue{
    double distance;
    double angle;
    maxValue();
};
SeamlessHybridExploration::maxValue::maxValue():distance(-DBL_MAX),angle(-DBL_MAX){};

struct SeamlessHybridExploration::preCalcResult{
    struct value{
        double distance;
        double angle;
        value();
        value(const double d, const double a);
    };
    geometry_msgs::Point point;
    // ExEnm::DuplicationStatus branchStatus;
    uint8_t branchStatus;
    std::vector<value> values;
    preCalcResult();
};
SeamlessHybridExploration::preCalcResult::preCalcResult(){};
SeamlessHybridExploration::preCalcResult::value::value(){};
SeamlessHybridExploration::preCalcResult::value::value(const double d, const double a):distance(d),angle(a){};

SeamlessHybridExploration::SeamlessHybridExploration()
    :robotArray_(new ExStc::subStruct<exploration_msgs::RobotInfoArray>("robot_array", 1))
    ,frontier_(new ExStc::subStruct<exploration_msgs::FrontierArray>("frontier", 1))
    // ,useFro_(new ExStc::pubStruct<exploration_msgs::FrontierArray>("useful_frontier", 1, true))
    // ,onMapFro_(new ExStc::pubStruct<exploration_msgs::FrontierArray>("on_map_frontier", 1, true))
    ,pp_(new ExpLib::PathPlanning<navfn::NavfnROS>("seamless_costmap","seamless_planner"))
    ,drs_(new dynamic_reconfigure::Server<exploration::seamless_hybrid_exploration_parameter_reconfigureConfig>(ros::NodeHandle("~/seamless_hybrid_exploration")))
    // ,ls_(new std::vector<ExStc::listStruct>())
    ,ba_(new exploration_msgs::BranchArray())
    ,fa_(new exploration_msgs::FrontierArray())
    ,ria_(new exploration_msgs::RobotInfoArray())
    ,ps_(new geometry_msgs::PoseStamped())
    ,mVal_(new maxValue()){
    loadParams();
    SensorBasedExploration::drs_->clearCallback();
    SeamlessHybridExploration::drs_->setCallback(boost::bind(&SeamlessHybridExploration::dynamicParamsCB,this, _1, _2));
}

SeamlessHybridExploration::~SeamlessHybridExploration(){
    if(OUTPUT_SHE_PARAMETERS) outputParams();
}

// bool SeamlessHybridExploration::decideGoal(geometry_msgs::PointStamped& goal, const std::vector<ExStc::listStruct>& ls, const geometry_msgs::PoseStamped& pose){
bool SeamlessHybridExploration::decideGoal(geometry_msgs::PointStamped& goal, const exploration_msgs::BranchArray& ba, const geometry_msgs::PoseStamped& pose){
    if(robotArray_->q.callOne(ros::WallDuration(1))){
        ROS_ERROR_STREAM("Can't read robot_array or don't find robot_array"); 
        return false;
    }
    if(frontier_->q.callOne(ros::WallDuration(1))){
        ROS_ERROR_STREAM("Can't read frontier or don't find frontier"); 
        return false;
    }

    *ps_ = pose;
    // *ls_ = ls;
    *ba_ = ba;
    *fa_ = frontier_->data;
    *ria_ = robotArray_->data;

    // if(!filter(*ls_, *fa_, *ria_)) return false;
    if(!filter(*ba_, *fa_, *ria_)) return false;

    return decideGoal(goal);
}

bool SeamlessHybridExploration::decideGoal(geometry_msgs::PointStamped& goal){
    // 事前計算
    // preCalc(*ls_, *fa_, *ria_, *ps_);
    preCalc(*ba_, *fa_, *ria_, *ps_);

    // 評価計算
    auto evaluation = [this](const double d, const double a){return DISTANCE_WEIGHT * d / mVal_->distance + DIRECTION_WEIGHT * a / mVal_->angle;};

    double minE = DBL_MAX;

    for(int m=0,me=ownPreCalc_.size();m!=me;++m){
        double e = 1;
        for(int i=0,ie=ownPreCalc_[m].values.size();i!=ie;++i){
            double subE = 0;
            if(otherPreCalc_.size()==0) subE = DBL_MAX;
            else for(const auto& opc : otherPreCalc_) subE += evaluation(opc.values[i].distance, opc.values[i].angle);
            e *= evaluation(ownPreCalc_[m].values[i].distance, ownPreCalc_[m].values[i].angle) + (OTHER_ROBOT_WEIGHT/subE);
        }
        // ROS_DEBUG_STREAM("position : (" << mainRobotInfo[m].robot.coordinate.x << "," << mainRobotInfo[m].robot.coordinate.y << "), sum : " << e);
        // if(ownPreCalc_[m].branchStatus==ExEnm::DuplicationStatus::OLDER) e *= DUPLICATE_COEFF;
        // else if(ownPreCalc_[m].branchStatus==ExEnm::DuplicationStatus::ON_MAP) e *= ON_MAP_COEFF;
        if(ownPreCalc_[m].branchStatus==exploration_msgs::Branch::OLDER_DUPLICATION) e *= DUPLICATE_COEFF;
        else if(ownPreCalc_[m].branchStatus==exploration_msgs::Branch::ON_MAP) e *= ON_MAP_COEFF;
        ROS_DEBUG_STREAM("position : (" << ownPreCalc_[m].point.x << "," << ownPreCalc_[m].point.y << "), sum : " << e);
        if(e < minE){
            minE = std::move(e);
            // goal = mainRobotInfo[m].robot.coordinate;
            goal.header.frame_id = ps_->header.frame_id;
            goal.header.stamp = ros::Time::now();
            goal.point = ownPreCalc_[m].point;
            goal_->pub.publish(goal);
            if(m == me -1) return false;
        }
    }
    *lastGoal_ = goal.point;
    return true;
}

// bool SeamlessHybridExploration::filter(std::vector<ExStc::listStruct>& ls, exploration_msgs::FrontierArray& fa, exploration_msgs::RobotInfoArray& ria){
bool SeamlessHybridExploration::filter(exploration_msgs::BranchArray& ba, exploration_msgs::FrontierArray& fa, exploration_msgs::RobotInfoArray& ria){
    //分岐領域のフィルタ
    // ROS_INFO_STREAM("before branches size : " << ls.size());
    // auto lsRemove = std::remove_if(ls.begin(),ls.end(),[this](ExStc::listStruct& l){return l.duplication == ExEnm::DuplicationStatus::NEWER;});
	// ls.erase(std::move(lsRemove),ls.end());
    // ROS_INFO_STREAM("after branches size : " << ls.size());

    ROS_INFO_STREAM("before branches size : " << ba.branches.size());
    auto baRemove = std::remove_if(ba.branches.begin(),ba.branches.end(),[this](exploration_msgs::Branch& b){return b.status == exploration_msgs::Branch::NEWER_DUPLICATION;});
	ba.branches.erase(std::move(baRemove),ba.branches.end());
    ROS_INFO_STREAM("after branches size : " << ba.branches.size());

    // if(ls.size()==0) return false;
    if(ba.branches.size()==0) return false;

    //フロンティア領域のフィルタ

    ROS_INFO_STREAM("before frontiers size : " << fa.frontiers.size());
    // 周囲の地図が出来ているフロンティアの削除
    // exploration_msgs::FrontierArray omf;
    // omf.header.frame_id = fa.header.frame_id;
    // if(ON_MAP_FRONTIER_DETECTION && !map_->q.callOne(ros::WallDuration(1))){
    //     omf.frontiers.reserve(fa.frontiers.size());
    //     std::vector<std::vector<int8_t>> map2d = ExUtl::mapArray1dTo2d(map_->data.data,map_->data.info);
    //     auto faRemove1 = std::remove_if(fa.frontiers.begin(),fa.frontiers.end(),[&,this](exploration_msgs::Frontier& f){
    //         ExStc::mapSearchWindow msw(f.point,map_->data.info,OMF_MAP_WINDOW_X,OMF_MAP_WINDOW_Y);
    //         int c = 0;
    //         for(int y=msw.top,ey=msw.bottom+1;y!=ey;++y){
    //             for(int x=msw.left,ex=msw.right+1;x!=ex;++x){
    //                 if(map2d[x][y] >= 0) ++c;
    //             }
    //         }
    //         if((double)c/(msw.width*msw.height)>ON_MAP_FRONTIER_RATE){
    //             omf.frontiers.emplace_back(f);
    //             return true;
    //         }
    //         else return false;
    //     });
    //     fa.frontiers.erase(std::move(faRemove1),fa.frontiers.end());
    //     omf.header.stamp = ros::Time::now();
    //     onMapFro_->pub.publish(omf);
    //     ROS_INFO_STREAM("after frontiers size 1 : " << fa.frontiers.size());
    // }
    // else{
    //     omf.header.stamp = ros::Time::now();
    //     onMapFro_->pub.publish(omf);
    // }

    fa.frontiers.erase(std::remove_if(fa.frontiers.begin(),fa.frontiers.end(),[](exploration_msgs::Frontier& f){
        return f.status == exploration_msgs::Frontier::ON_MAP;
    }),fa.frontiers.end());
    ROS_INFO_STREAM("after frontiers size 1 : " << fa.frontiers.size());

    // 分散が小さいかつ共分散も小さいものを削除
    // auto faRemove2 = std::remove_if(fa.frontiers.begin(),fa.frontiers.end(),[this](exploration_msgs::Frontier& f){return ((f.variance.x>f.variance.y ? f.variance.x : f.variance.y) < VARIANCE_THRESHOLD) && std::abs(f.covariance) < COVARIANCE_THRESHOLD;});
    // auto faRemove2 = std::remove_if(fa.frontiers.begin(),fa.frontiers.end(),[this](exploration_msgs::Frontier& f){
    //     return std::max(f.variance.x,f.variance.y) < VARIANCE_MIN_THRESHOLD || (std::max(f.variance.x,f.variance.y) < VARIANCE_THRESHOLD && std::abs(f.covariance) < COVARIANCE_THRESHOLD);
    // });
	// fa.frontiers.erase(std::move(faRemove2),fa.frontiers.end());
    fa.frontiers.erase(std::remove_if(fa.frontiers.begin(),fa.frontiers.end(),[](exploration_msgs::Frontier& f){
        return f.status == exploration_msgs::Frontier::NOT_USEFUL;
    }),fa.frontiers.end());

    ROS_INFO_STREAM("after frontiers size 2 : " << fa.frontiers.size());

    // useFro_->pub.publish(fa);
    if(fa.frontiers.size()==0) return false;

    //ロボットリストのフィルタ

    ROS_INFO_STREAM("before robotList size : " << ria.info.size());
    auto riaRemove = std::remove_if(ria.info.begin(),ria.info.end(),[this](exploration_msgs::RobotInfo& ri){return ri.name == "/" + ROBOT_NAME;});
	ria.info.erase(std::move(riaRemove),ria.info.end());
    ROS_INFO_STREAM("after robotList size : " << ria.info.size());
    
    return true;
}

// void SeamlessHybridExploration::preCalc(const std::vector<ExStc::listStruct>& ls, const exploration_msgs::FrontierArray& fa, const exploration_msgs::RobotInfoArray& ria, const geometry_msgs::PoseStamped& pose){
void SeamlessHybridExploration::preCalc(const exploration_msgs::BranchArray& ba, const exploration_msgs::FrontierArray& fa, const exploration_msgs::RobotInfoArray& ria, const geometry_msgs::PoseStamped& pose){
    ownPreCalc_ = std::vector<preCalcResult>();
    otherPreCalc_ = std::vector<preCalcResult>();
    *mVal_ = maxValue();

    // auto calc = [&,this](const geometry_msgs::Point& p,const Eigen::Vector2d& v1, ExEnm::DuplicationStatus branchStatus){
    auto calc = [&,this](const geometry_msgs::Point& p,const Eigen::Vector2d& v1, uint8_t branchStatus){
        SeamlessHybridExploration::preCalcResult pcr;
        pcr.point = p;
        pcr.branchStatus = branchStatus;
        // Eigen::Vector2d v1 = ExCov::qToVector2d(ps.orientation);
        pcr.values.reserve(fa.frontiers.size());
        for(const auto& f : fa.frontiers){
            // 目標地点での向きをpathの最後の方の移動で決めたい
            Eigen::Vector2d v2;
            double distance;
            if(!pp_->getDistanceAndVec(ExCov::pointToPoseStamped(p,pose.header.frame_id),ExCov::pointToPoseStamped(f.point,fa.header.frame_id),distance,v2)){
                v2 = Eigen::Vector2d(f.point.x - p.x, f.point.y - p.y).normalized();
                // if(!pp_->getDistance(ExCov::pointToPoseStamped(p,pose.header.frame_id),ExCov::pointToPoseStamped(f.point,fa.header.frame_id),distance)){
                    //最終手段で直線距離を計算
                    distance = Eigen::Vector2d(f.point.x - p.x, f.point.y - p.y).norm();
                // }                
            }
            double angle = std::abs(acos(v1.dot(v2)));
            pcr.values.emplace_back(preCalcResult::value(distance,angle));
            if(angle > mVal_->angle) mVal_->angle = std::move(angle);
            if(distance > mVal_->distance) mVal_->distance = std::move(distance);
        }
        return pcr;
    };

    // ownPreCalc_.reserve(ls.size()+1);
    ownPreCalc_.reserve(ba.branches.size()+1);
    otherPreCalc_.reserve(ria.info.size());

    // 分岐領域の計算
    double forward = 0;
    // for(const auto& l : ls){
    for(const auto& b : ba.branches){
        double distance;
        Eigen::Vector2d v1;
        if(!pp_->getDistanceAndVec(pose,ExCov::pointToPoseStamped(b.point,pose.header.frame_id),distance,v1)){
            v1 = Eigen::Vector2d(b.point.x - pose.pose.position.x, b.point.y - pose.pose.position.y);
            distance = v1.lpNorm<1>();
            v1.normalize();
        }
        ownPreCalc_.emplace_back(calc(b.point,v1,b.status));
        forward += distance;
    }
    // forward /= ls.size();
    forward /= ba.branches.size();
    // 直進時の計算
    Eigen::Vector2d fwdV1 = ExCov::qToVector2d(pose.pose.orientation); 
    // ownPreCalc_.emplace_back(calc(ExCov::vector2dToPoint(Eigen::Vector2d(pose.pose.position.x,pose.pose.position.y)+forward*fwdV1),fwdV1,ExEnm::DuplicationStatus::NOT_DUPLECATION));
    ownPreCalc_.emplace_back(calc(ExCov::vector2dToPoint(Eigen::Vector2d(pose.pose.position.x,pose.pose.position.y)+forward*fwdV1),fwdV1,exploration_msgs::Branch::NORMAL));
    // 他のロボットに関する計算
    // for(const auto& ri : ria.info) otherPreCalc_.emplace_back(calc(ri.pose.position,ExCov::qToVector2d(ri.pose.orientation),ExEnm::DuplicationStatus::NOT_DUPLECATION));
    for(const auto& ri : ria.info) otherPreCalc_.emplace_back(calc(ri.pose.position,ExCov::qToVector2d(ri.pose.orientation),exploration_msgs::Branch::NORMAL));
}

bool SeamlessHybridExploration::getGoalAF(geometry_msgs::PointStamped& goal){
    // 面積の条件で切り替えた後の目標
    if(frontier_->q.callOne(ros::WallDuration(1))){
        ROS_ERROR_STREAM("Can't read frontier or don't find frontier"); 
        return false;
    }

    if(pose_->q.callOne(ros::WallDuration(1))){
        ROS_ERROR_STREAM("Can't read pose");
        return false;
    }

    std::vector<exploration_msgs::Frontier> frontiers(frontier_->data.frontiers);

    if(CANCELED_GOAL_EFFECT && !canceled_->q.callOne(ros::WallDuration(1)) && canceled_->data.points.size()!=0){
		frontiers.erase(std::remove_if(frontiers.begin(),frontiers.end(),[this](exploration_msgs::Frontier& f){
            for(const auto& c : canceled_->data.points){
                if(Eigen::Vector2d(f.point.x - c.x, f.point.y - c.y).norm()<CANCELED_GOAL_TOLERANCE) return true;
            }
            return false;
        }),frontiers.end());
    }

    if(frontiers.size() == 0){
        ROS_ERROR_STREAM("Frontier array became empty");
        return false;
    }

    std::vector<std::tuple<double,double,geometry_msgs::Point,uint8_t>> distAng; // distance, angle
    std::vector<std::pair<geometry_msgs::Point,double>> valNormal;
    std::vector<std::pair<geometry_msgs::Point,double>> valNotUseful;
    std::vector<std::pair<geometry_msgs::Point,double>> valOnMap;

    valNormal.reserve(frontiers.size());
    valNotUseful.reserve(frontiers.size());
    valOnMap.reserve(frontiers.size());

    // frontierまでの距離と角度の最大
    double distMax = -DBL_MAX;
    double angMax = -DBL_MAX;

    // usefulなやつの中で距離(path)の近いやつが良い -> frontierと距離と角度の重みのやつ
    // 無いときはnot_usefulなやつで
    Eigen::Vector2d v1 = ExCov::qToVector2d(pose_->data.pose.orientation);
    for(const auto& f : frontiers){
        // 目標地点での向きをpathの最後の方の移動で決めたい
        Eigen::Vector2d v2;
        double distance;
        if(!pp_->getDistanceAndVec(pose_->data,ExCov::pointToPoseStamped(f.point,frontier_->data.header.frame_id),distance,v2)){
            v2 = Eigen::Vector2d(f.point.x - pose_->data.pose.position.x, f.point.y - pose_->data.pose.position.y).normalized();
            distance = Eigen::Vector2d(f.point.x - pose_->data.pose.position.x, f.point.y - pose_->data.pose.position.y).norm();       
        }
        double angle = std::abs(acos(v1.dot(v2)));
        distAng.emplace_back(std::make_tuple(distance, angle ,f.point, f.status));
        if(angle > angMax) angMax = std::move(angle);
        if(distance > distMax) distMax = std::move(distance);
    }

    auto evaluation = [this](double d, double dMax, double a, double aMax){return DISTANCE_WEIGHT * d / dMax + DIRECTION_WEIGHT * a / aMax;};

    for(const auto& da : distAng){
        switch (std::get<3>(da)){
        case exploration_msgs::Frontier::NORMAL:
            valNormal.emplace_back(std::make_pair(std::get<2>(da),evaluation(std::get<0>(da),distMax,std::get<1>(da),angMax)));
            break;
        case exploration_msgs::Frontier::NOT_USEFUL:
            valNotUseful.emplace_back(std::make_pair(std::get<2>(da),evaluation(std::get<0>(da),distMax,std::get<1>(da),angMax)));
            break;
        default:
            valOnMap.emplace_back(std::make_pair(std::get<2>(da),evaluation(std::get<0>(da),distMax,std::get<1>(da),angMax)));
            break;
        }
    }

    if(!valNormal.empty()){
        std::sort(valNormal.begin(),valNormal.end(),[](const std::pair<geometry_msgs::Point,double>& l, const std::pair<geometry_msgs::Point,double>& r){return l.second > r.second;});
        goal.point = valNormal[0].first;
    }
    else if(!valNotUseful.empty()){
        std::sort(valNotUseful.begin(),valNotUseful.end(),[](const std::pair<geometry_msgs::Point,double>& l, const std::pair<geometry_msgs::Point,double>& r){return l.second > r.second;});
        goal.point = valNotUseful[0].first;
    }
    else if(!valOnMap.empty()){
        std::sort(valOnMap.begin(),valOnMap.end(),[](const std::pair<geometry_msgs::Point,double>& l, const std::pair<geometry_msgs::Point,double>& r){return l.second > r.second;});
        goal.point = valOnMap[0].first;
    }
    else return false;

    goal.header.frame_id = frontier_->data.header.frame_id;
    goal.header.stamp = ros::Time::now();
    goal_->pub.publish(goal);


    return true;
}

void SeamlessHybridExploration::simBridge(std::vector<geometry_msgs::Pose>& r, std::vector<geometry_msgs::Point>& b, std::vector<geometry_msgs::Point>& f){
    static ExpLib::PathPlanning<navfn::NavfnROS> pp("seamsim_goal_costmap","seamsim_goal_path");

    ros::NodeHandle nh("~/mulsim");
    std::string FRAME_ID;
    int SIMULATE_ROBOT_INDEX;

    nh.param<std::string>("frame_id", FRAME_ID, "map");
    nh.param<int>("simulate_robot_index", SIMULATE_ROBOT_INDEX, 1); 

    if(SIMULATE_ROBOT_INDEX > r.size()) SIMULATE_ROBOT_INDEX = 1;

    geometry_msgs::PoseStamped ps;
    // std::vector<ExStc::listStruct> ls;
    exploration_msgs::BranchArray ba;
    exploration_msgs::FrontierArray fa;
    exploration_msgs::RobotInfoArray ria;

    // ls 格納
    // ls.resize(b.size());
    // for(int i=0,ie=b.size();i!=ie;++i) ls[i].point = b[i];
    // *ls_ = ls;
    ba.branches.resize(b.size());
    for(int i=0,ie=b.size();i!=ie;++i) ba.branches[i].point = b[i];
    *ba_ = ba;

    // fa 格納
    fa.frontiers.resize(f.size());
    fa.header.frame_id = FRAME_ID;
    for(int i=0,ie=f.size();i!=ie;++i) fa.frontiers[i].point = f[i];
    *fa_ = fa;

    // ps 格納
    ps.header.frame_id = FRAME_ID;
    ps.pose = r[SIMULATE_ROBOT_INDEX-1];
    *ps_ = ps;

    // ria 格納 
    ria.info.resize(r.size()-1);
    for(int i=0,ie=r.size(),ix=0;i!=ie;++i) {
        if(i+1 == SIMULATE_ROBOT_INDEX) continue;
        ria.info[ix++].pose = r[i];
    }
    *ria_ = ria;

    geometry_msgs::PointStamped goal;
    decideGoal(goal);

    pp.createPath(*ps_,ExCov::pointStampedToPoseStamped(goal));
}

void SeamlessHybridExploration::loadParams(void){
    ros::NodeHandle nh("~/seamless_hybrid_exploration");
    // dynamic parameters
    nh.param<bool>("last_goal_effect", LAST_GOAL_EFFECT, true);
    nh.param<double>("last_goal_tolerance", LAST_GOAL_TOLERANCE, 1.0);
    nh.param<bool>("canceled_goal_effect", CANCELED_GOAL_EFFECT, true);
    nh.param<double>("canceled_goal_tolerance", CANCELED_GOAL_TOLERANCE, 0.5);
    // nh.param<bool>("on_map_branch_detection", ON_MAP_BRANCH_DETECTION, true);
    // nh.param<double>("omb_map_window_x", OMB_MAP_WINDOW_X, 1.0);
    // nh.param<double>("omb_map_window_y", OMB_MAP_WINDOW_Y, 1.0);
    // nh.param<double>("on_map_branch_rate", ON_MAP_BRANCH_RATE, 0.5);
    // nh.param<bool>("duplicate_detection", DUPLICATE_DETECTION, true);
    // nh.param<double>("duplicate_tolerance", DUPLICATE_TOLERANCE, 1.5);
    // nh.param<double>("log_current_time", LOG_CURRENT_TIME, 10);
    // nh.param<double>("newer_duplication_threshold", NEWER_DUPLICATION_THRESHOLD, 100);
    // nh.param<bool>("on_map_frontier_detection", ON_MAP_FRONTIER_DETECTION, true);
    // nh.param<double>("omf_map_window_x", OMF_MAP_WINDOW_X, 1.0);
    // nh.param<double>("omf_map_window_y", OMF_MAP_WINDOW_Y, 1.0);
    // nh.param<double>("on_map_frontier_rate", ON_MAP_FRONTIER_RATE, 0.5);
    nh.param<double>("direction_weight", DIRECTION_WEIGHT, 1.5);
    nh.param<double>("distance_weight", DISTANCE_WEIGHT, 2.5);
    // nh.param<double>("variance_threshold", VARIANCE_THRESHOLD, 1.5);
    // nh.param<double>("variance_min_threshold", VARIANCE_MIN_THRESHOLD, 0.1);
    // nh.param<double>("covariance_threshold", COVARIANCE_THRESHOLD, 0.7);
    nh.param<double>("other_robot_weight", OTHER_ROBOT_WEIGHT, 1.0);
    nh.param<double>("duplicate_coeff", DUPLICATE_COEFF, 1.2);
    nh.param<double>("on_map_coeff", ON_MAP_COEFF, 1.1);
    // static parameters
    nh.param<std::string>("robot_name", ROBOT_NAME, "robot1");
    nh.param<std::string>("she_parameter_file_path",SHE_PARAMETER_FILE_PATH,"she_last_parameters.yaml");
    nh.param<bool>("output_she_parameters",OUTPUT_SHE_PARAMETERS,true);
}

void SeamlessHybridExploration::dynamicParamsCB(exploration::seamless_hybrid_exploration_parameter_reconfigureConfig &cfg, uint32_t level){
    LAST_GOAL_EFFECT = cfg.last_goal_effect;
    LAST_GOAL_TOLERANCE = cfg.last_goal_tolerance;
    CANCELED_GOAL_EFFECT = cfg.canceled_goal_effect;
    CANCELED_GOAL_TOLERANCE = cfg.canceled_goal_tolerance;
    // ON_MAP_BRANCH_DETECTION = cfg.on_map_branch_detection;
    // OMB_MAP_WINDOW_X = cfg.omb_map_window_x;
    // OMB_MAP_WINDOW_Y = cfg.omb_map_window_y;
    // ON_MAP_BRANCH_RATE = cfg.on_map_branch_rate;
    // DUPLICATE_DETECTION = cfg.duplicate_detection;
    // DUPLICATE_TOLERANCE = cfg.duplicate_tolerance;
    // LOG_CURRENT_TIME = cfg.log_current_time;
    // NEWER_DUPLICATION_THRESHOLD = cfg.newer_duplication_threshold;
    // ON_MAP_FRONTIER_DETECTION = cfg.on_map_frontier_detection;
    // OMF_MAP_WINDOW_X = cfg.omf_map_window_x;
    // OMF_MAP_WINDOW_Y = cfg.omf_map_window_y;
    // ON_MAP_FRONTIER_RATE = cfg.on_map_frontier_rate;
    DISTANCE_WEIGHT = cfg.distance_weight;
    DIRECTION_WEIGHT = cfg.direction_weight;
    // VARIANCE_THRESHOLD = cfg.variance_threshold;
    // VARIANCE_MIN_THRESHOLD = cfg.variance_min_threshold;
    // COVARIANCE_THRESHOLD = cfg.covariance_threshold;
    OTHER_ROBOT_WEIGHT = cfg.other_robot_weight;
    DUPLICATE_COEFF = cfg.duplicate_coeff;
    ON_MAP_COEFF = cfg.on_map_coeff;
}

void SeamlessHybridExploration::outputParams(void){
    std::cout << "writing she last parameters ... ..." << std::endl;
    std::ofstream ofs(SHE_PARAMETER_FILE_PATH);

    if(ofs) std::cout << "she param file open succeeded" << std::endl;
    else {
        std::cout << "she param file open failed" << std::endl;
        return;
    }

    ofs << "last_goal_effect: " << (LAST_GOAL_EFFECT ? "true" : "false") << std::endl;
    ofs << "last_goal_tolerance: " << LAST_GOAL_TOLERANCE << std::endl;
    ofs << "canceled_goal_effect: " << (CANCELED_GOAL_EFFECT ? "true" : "false") << std::endl;
    ofs << "canceled_goal_tolerance: " << CANCELED_GOAL_TOLERANCE << std::endl;
    // ofs << "on_map_branch_detection: " << (ON_MAP_BRANCH_DETECTION ? "true" : "false") << std::endl;
    // ofs << "omb_map_window_x: " << OMB_MAP_WINDOW_X << std::endl;
    // ofs << "omb_map_window_y: " << OMB_MAP_WINDOW_Y << std::endl;
    // ofs << "on_map_branch_rate: " << ON_MAP_BRANCH_RATE << std::endl;
    // ofs << "duplicate_detection: " << (DUPLICATE_DETECTION ? "true" : "false") << std::endl;
    // ofs << "duplicate_tolerance: " << DUPLICATE_TOLERANCE << std::endl;
    // ofs << "log_current_time: " << LOG_CURRENT_TIME << std::endl;
    // ofs << "newer_duplication_threshold: " << NEWER_DUPLICATION_THRESHOLD << std::endl;
    // ofs << "on_map_frontier_detection: " << (ON_MAP_FRONTIER_DETECTION ? "true" : "false") << std::endl;
    // ofs << "omf_map_window_x: " << OMF_MAP_WINDOW_X << std::endl;
    // ofs << "omf_map_window_y: " << OMF_MAP_WINDOW_Y << std::endl;
    // ofs << "on_map_frontier_rate: " << ON_MAP_FRONTIER_RATE << std::endl;
    ofs << "distance_weight: " << DISTANCE_WEIGHT << std::endl;
    ofs << "direction_weight: " << DIRECTION_WEIGHT << std::endl;
    // ofs << "variance_threshold: " << VARIANCE_THRESHOLD << std::endl;
    // ofs << "variance_min_threshold: " << VARIANCE_MIN_THRESHOLD << std::endl;
    // ofs << "covariance_threshold: " << COVARIANCE_THRESHOLD << std::endl;
    ofs << "other_robot_weight: " << OTHER_ROBOT_WEIGHT << std::endl;
    ofs << "duplicate_coeff: " << DUPLICATE_COEFF << std::endl;
    ofs << "on_map_coeff: " << ON_MAP_COEFF << std::endl;
 }