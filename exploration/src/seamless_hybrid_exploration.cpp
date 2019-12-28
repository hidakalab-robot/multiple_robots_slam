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
    ROS_INFO_STREAM("before branches size : " << ba.branches.size());
    auto baRemove = std::remove_if(ba.branches.begin(),ba.branches.end(),[this](exploration_msgs::Branch& b){return b.status == exploration_msgs::Branch::NEWER_DUPLICATION;});
	ba.branches.erase(std::move(baRemove),ba.branches.end());
    ROS_INFO_STREAM("after branches size : " << ba.branches.size());

    // if(ls.size()==0) return false;
    if(ba.branches.size()==0) return false;

    //フロンティア領域のフィルタ

    ROS_INFO_STREAM("before frontiers size : " << fa.frontiers.size());

    fa.frontiers.erase(std::remove_if(fa.frontiers.begin(),fa.frontiers.end(),[](exploration_msgs::Frontier& f){
        return f.status == exploration_msgs::Frontier::ON_MAP;
    }),fa.frontiers.end());
    ROS_INFO_STREAM("after frontiers size 1 : " << fa.frontiers.size());

    // 分散が小さいかつ共分散も小さいものを削除
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

bool SeamlessHybridExploration::forwardTargetDetection(void){
    // 前方に目標候補が存在するかを見る

    ROS_DEBUG_STREAM("function : forwardTargetDetection");  

    if(pose_->q.callOne(ros::WallDuration(1))){
        ROS_ERROR_STREAM("Can't read pose");
        return false;
    }

    branch_->q.callOne(ros::WallDuration(1));
    frontier_->q.callOne(ros::WallDuration(1));

    // 自分の姿勢と自分の位置からターゲット候補の位置の角度を計算
    Eigen::Vector2d v1 = ExCov::qToVector2d(pose_->data.pose.orientation);

    for(const auto& b : branch_->data.branches){
        Eigen::Vector2d v2;
        if(!pp_->getVec(pose_->data,ExCov::pointToPoseStamped(b.point,branch_->data.header.frame_id),v2)){
            v2 = Eigen::Vector2d(b.point.x - pose_->data.pose.position.x, b.point.y - pose_->data.pose.position.y).normalized();   
        }
        if(std::abs(acos(v1.dot(v2)))<M_PI/2){
            ROS_INFO_STREAM("detected forward target (bracnch) : (" << b.point.x << ", " << b.point.y << ")");
            return true;
        }
    }

    for(const auto& f : frontier_->data.frontiers){
        Eigen::Vector2d v2;
        if(!pp_->getVec(pose_->data,ExCov::pointToPoseStamped(f.point,branch_->data.header.frame_id),v2)){
            v2 = Eigen::Vector2d(f.point.x - pose_->data.pose.position.x, f.point.y - pose_->data.pose.position.y).normalized();   
        }
        if(std::abs(acos(v1.dot(v2)))<M_PI/2){
            ROS_INFO_STREAM("detected forward target (frontier) : (" << f.point.x << ", " << f.point.y << ")");
            return true;
        }
    }

    ROS_INFO_STREAM("don't detected forward target");

    return false;

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

    int STATUS_SIZE = 3;

    std::vector<std::tuple<double,double,double,geometry_msgs::Point,uint8_t>> distAng; // distance, angle, lastgoal, frontier, status
    // std::vector<std::vector<std::pair<geometry_msgs::Point,double>>> valList(STATUS_SIZE,std::vector<std::pair<geometry_msgs::Point,double>>());
    std::vector<std::pair<geometry_msgs::Point,double>> valList;

    // frontierまでの距離と角度の最大
    double distMax = -DBL_MAX;
    double angMax = -DBL_MAX;
    double lastMax = -DBL_MAX;

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
        double last = Eigen::Vector2d(lastGoal_->x-f.point.x,lastGoal_->y-f.point.y).norm();
        distAng.emplace_back(std::make_tuple(distance, angle ,last, f.point, f.status));
        if(angle > angMax) angMax = std::move(angle);
        if(distance > distMax) distMax = std::move(distance);
        if(last > lastMax) lastMax = std::move(last);
    }

    // double LAST_GOAL_WEIGHT = 1.2;

    auto evaluation = [STATUS_SIZE,this](double d, double dMax, double a, double aMax, double l, double lMax, uint8_t status){
        return (DISTANCE_WEIGHT * d / dMax + DIRECTION_WEIGHT * a / aMax + LAST_GOAL_WEIGHT * l / lMax)*((1.0+status)/STATUS_SIZE);
    };

    for(const auto& da : distAng)
        // valList[std::get<4>(da)].emplace_back(std::make_pair(std::get<3>(da),evaluation(std::get<0>(da),distMax,std::get<1>(da),angMax,std::get<2>(da),lastMax)));
        valList.emplace_back(std::make_pair(std::get<3>(da),evaluation(std::get<0>(da),distMax,std::get<1>(da),angMax,std::get<2>(da),lastMax,std::get<4>(da))));

    // for(auto&& vl : valList){
    //     if(!vl.empty()){
    //         std::sort(vl.begin(),vl.end(),[](const std::pair<geometry_msgs::Point,double>& l, const std::pair<geometry_msgs::Point,double>& r){return l.second < r.second;});
    //         goal.point = vl[0].first;
    //         break;
    //     }
    // }
    std::sort(valList.begin(),valList.end(),[](const std::pair<geometry_msgs::Point,double>& l, const std::pair<geometry_msgs::Point,double>& r){return l.second < r.second;});
    goal.point = valList[0].first;

    // 最後の目標と同じところだったら新しいゴールとして返さない
    if(Eigen::Vector2d(lastGoal_->x-goal.point.x,lastGoal_->y-goal.point.y).norm()< LAST_GOAL_TOLERANCE) return false;

    goal.header.frame_id = frontier_->data.header.frame_id;
    goal.header.stamp = ros::Time::now();
    goal_->pub.publish(goal);

    *lastGoal_ = goal.point;

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
    nh.param<double>("direction_weight", DIRECTION_WEIGHT, 1.5);
    nh.param<double>("distance_weight", DISTANCE_WEIGHT, 2.5);
    nh.param<double>("other_robot_weight", OTHER_ROBOT_WEIGHT, 1.0);
    nh.param<double>("duplicate_coeff", DUPLICATE_COEFF, 1.2);
    nh.param<double>("on_map_coeff", ON_MAP_COEFF, 1.1);
    nh.param<double>("last_goal_weight", LAST_GOAL_WEIGHT, 1.2);
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
    DISTANCE_WEIGHT = cfg.distance_weight;
    DIRECTION_WEIGHT = cfg.direction_weight;
    OTHER_ROBOT_WEIGHT = cfg.other_robot_weight;
    DUPLICATE_COEFF = cfg.duplicate_coeff;
    ON_MAP_COEFF = cfg.on_map_coeff;
    LAST_GOAL_WEIGHT = cfg.last_goal_weight;
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
    ofs << "distance_weight: " << DISTANCE_WEIGHT << std::endl;
    ofs << "direction_weight: " << DIRECTION_WEIGHT << std::endl;
    ofs << "other_robot_weight: " << OTHER_ROBOT_WEIGHT << std::endl;
    ofs << "duplicate_coeff: " << DUPLICATE_COEFF << std::endl;
    ofs << "on_map_coeff: " << ON_MAP_COEFF << std::endl;
    ofs << "last_goal_weight: " << LAST_GOAL_WEIGHT << std::endl;
 }