#ifndef SEAMLESS_HYBRID_EXPLORATION_HPP
#define SEAMLESS_HYBRID_EXPLORATION_HPP

#include <exploration/sensor_based_exploration.hpp>
#include <exploration_libraly/path_planning.hpp>
#include <navfn/navfn_ros.h>
#include <exploration_msgs/RobotInfoArray.h>
#include <exploration_msgs/FrontierArray.h>
#include <exploration_libraly/convert.hpp>

class SeamlessHybridExploration :public SensorBasedExploration
{
private:
    struct maxValue{
        double distance;
        double angle;
        maxValue():distance(-DBL_MAX),angle(-DBL_MAX){};
    };

    struct preCalcResult{
        struct value{
            double distance;
            double angle;
            value(){};
            value(const double d, const double a):distance(d),angle(a){};
        };
        geometry_msgs::Point point;
        std::vector<value> values;
        preCalcResult(){};
    };

    maxValue mVal_;

    double COVARIANCE_THRESHOLD;
    double VARIANCE_THRESHOLD;
    double DIRECTION_WEIGHT;
    double DISTANCE_WEIGHT;
    double OTHER_ROBOT_WEIGHT;
    std::string ROBOT_NAME;

    geometry_msgs::PoseStamped ps_;
    std::vector<ExpLib::listStruct> ls_;
    exploration_msgs::FrontierArray fa_;
    exploration_msgs::RobotInfoArray ria_;

    PathPlanning<navfn::NavfnROS> pp_;
    ExpLib::subStruct<exploration_msgs::RobotInfoArray> robotArray_;
    ExpLib::subStruct<exploration_msgs::FrontierArray> frontier_;

    std::vector<preCalcResult> ownPreCalc_;
    std::vector<preCalcResult> otherPreCalc_;

    bool decideGoal(geometry_msgs::PointStamped& goal);
    bool decideGoal(geometry_msgs::PointStamped& goal, const std::vector<ExpLib::listStruct>& ls, const geometry_msgs::PoseStamped& pose);
    bool filter(std::vector<ExpLib::listStruct>& ls, exploration_msgs::FrontierArray& fa, exploration_msgs::RobotInfoArray& ria);

    void preCalc(const std::vector<ExpLib::listStruct>& ls, const exploration_msgs::FrontierArray& fa, const exploration_msgs::RobotInfoArray& ria, const geometry_msgs::PoseStamped& pose);
public:
    SeamlessHybridExploration();
    void simBridge(std::vector<geometry_msgs::Pose>& r, std::vector<geometry_msgs::Point>& b, std::vector<geometry_msgs::Point>& f);
};

SeamlessHybridExploration::SeamlessHybridExploration()
    :robotArray_("robotArray", 1)
    ,frontier_("frontier", 1)
    ,pp_("global_costmap","NavfnROS"){

    ros::NodeHandle p("~");
    p.param<double>("covariance_threshold", COVARIANCE_THRESHOLD, 0.7);
    p.param<double>("variance_threshold", VARIANCE_THRESHOLD, 1.5);
    p.param<double>("direction_weight", DIRECTION_WEIGHT, 1.5);
    p.param<double>("distance_weight", DISTANCE_WEIGHT, 2.5);
    p.param<double>("other_robot_weight", OTHER_ROBOT_WEIGHT, 1.0);
    p.param<std::string>("robot_name", ROBOT_NAME, "robot1");
}

bool SeamlessHybridExploration::decideGoal(geometry_msgs::PointStamped& goal, const std::vector<ExpLib::listStruct>& ls, const geometry_msgs::PoseStamped& pose){
    if(this->robotArray_.q.callOne(ros::WallDuration(1))){
        return false;
    }
    if(this->frontier_.q.callOne(ros::WallDuration(1))){
        return false;
    }

    this->ps_ = pose;
    this->ls_ = ls;
    this->fa_ = this->frontier_.data;
    this->ria_ = this->robotArray_.data;

    if(!this->filter(ls_, fa_, ria_)) return false;

    return this->decideGoal(goal);

    // preCalc(ls_, fa_, ria_, pose);

    // // 評価計算
    // auto evaluation = [this](const double d, const double a){return DISTANCE_WEIGHT * d / mVal.distance + DIRECTION_WEIGHT * a / mVal.angle;};

    // double minE = DBL_MAX;

    // for(int m=0,me=ownPreCalc.size();m!=me;++m){
    //     double e = 1;
    //     for(int i=0,ie=ownPreCalc[m].values.size();i!=ie;++i){
    //         double subE = 0;
    //         if(otherPreCalc.size()==0) subE = DBL_MAX;
    //         else for(const auto& opc : otherPreCalc) subE += evaluation(opc.values[i].distance, opc.values[i].angle);
    //         e *= evaluation(ownPreCalc[m].values[i].distance, ownPreCalc[m].values[i].angle) + (OTHER_ROBOT_WEIGHT/subE);
    //     }
    //     // ROS_DEBUG_STREAM("position : (" << mainRobotInfo[m].robot.coordinate.x << "," << mainRobotInfo[m].robot.coordinate.y << "), sum : " << e);
    //     ROS_DEBUG_STREAM("position : (" << ownPreCalc[m].point.x << "," << ownPreCalc[m].point.y << "), sum : " << e);
    //     if(e < minE){
    //         minE = std::move(e);
    //         // goal = mainRobotInfo[m].robot.coordinate;
    //         goal.header.frame_id = pose.header.frame_id;
    //         goal.header.stamp = ros::Time::now();
    //         goal.point = ownPreCalc[m].point;
    //         goal_.pub.publish(goal);
    //         if(m == me -1) return false;
    //     }
    // }
    // this->lastGoal = goal.point;
    // return true;
}

bool SeamlessHybridExploration::decideGoal(geometry_msgs::PointStamped& goal){
    // 事前計算
    preCalc(ls_, fa_, ria_, ps_);

    // 評価計算
    auto evaluation = [this](const double d, const double a){return DISTANCE_WEIGHT * d / mVal_.distance + DIRECTION_WEIGHT * a / mVal_.angle;};

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
        ROS_DEBUG_STREAM("position : (" << ownPreCalc_[m].point.x << "," << ownPreCalc_[m].point.y << "), sum : " << e);
        if(e < minE){
            minE = std::move(e);
            // goal = mainRobotInfo[m].robot.coordinate;
            goal.header.frame_id = ps_.header.frame_id;
            goal.header.stamp = ros::Time::now();
            goal.point = ownPreCalc_[m].point;
            goal_.pub.publish(goal);
            if(m == me -1) return false;
        }
    }
    this->lastGoal = goal.point;
    return true;
}

bool SeamlessHybridExploration::filter(std::vector<ExpLib::listStruct>& ls, exploration_msgs::FrontierArray& fa, exploration_msgs::RobotInfoArray& ria){
    //分岐領域のフィルタ
    ROS_INFO_STREAM("before branches size : " << ls.size());
    auto lsRemove = std::remove_if(ls.begin(),ls.end(),[this](ExpLib::listStruct& l){return l.duplication == ExpLib::DuplicationStatus::NEWER;});
	ls.erase(std::move(lsRemove),ls.end());
    ROS_INFO_STREAM("after branches size : " << ls.size());

    if(ls.size()==0) return false;

    //フロンティア領域のフィルタ

    ROS_INFO_STREAM("before frontiers size : " << fa.frontiers.size());
    // 分散が小さいかつ共分散も小さいものを削除
    auto faRemove = std::remove_if(fa.frontiers.begin(),fa.frontiers.end(),[this](exploration_msgs::Frontier& f){return (f.variance.x>f.variance.y ? f.variance.x : f.variance.y < VARIANCE_THRESHOLD) && std::abs(f.covariance) < COVARIANCE_THRESHOLD;});
	fa.frontiers.erase(std::move(faRemove),fa.frontiers.end());
    ROS_INFO_STREAM("after frontiers size : " << fa.frontiers.size());

    if(fa.frontiers.size()==0) return false;

    //ロボットリストのフィルタ

    ROS_INFO_STREAM("before robotList size : " << ria.info.size());
    auto riaRemove = std::remove_if(ria.info.begin(),ria.info.end(),[this](exploration_msgs::RobotInfo& ri){return ri.name == "/" + ROBOT_NAME;});
	ria.info.erase(std::move(riaRemove),ria.info.end());
    ROS_INFO_STREAM("after robotList size : " << ria.info.size());
    
    return true;
}

void SeamlessHybridExploration::preCalc(const std::vector<ExpLib::listStruct>& ls, const exploration_msgs::FrontierArray& fa, const exploration_msgs::RobotInfoArray& ria, const geometry_msgs::PoseStamped& pose){
    this->ownPreCalc_ = std::vector<preCalcResult>();
    this->otherPreCalc_ = std::vector<preCalcResult>();
    this->mVal_ = maxValue();

    auto calc = [&,this](const geometry_msgs::Point& p,const Eigen::Vector2d& v1){
        SeamlessHybridExploration::preCalcResult pcr;
        pcr.point = p;
        // Eigen::Vector2d v1 = ExpLib::qToVector2d(ps.orientation);
        pcr.values.reserve(fa.frontiers.size());
        for(const auto& f : fa.frontiers){
            // 目標地点での向きをpathの最後の方の移動で決めたい
            Eigen::Vector2d v2;
            double distance;
            if(!pp_.getDistanceAndVec(ExpLib::pointToPoseStamped(p,pose.header.frame_id),ExpLib::pointToPoseStamped(f.point,fa.header.frame_id),distance,v2)){
                v2 = Eigen::Vector2d(f.point.x - p.x, f.point.y - p.y).normalized();
                if(!pp_.getDistance(ExpLib::pointToPoseStamped(p,pose.header.frame_id),ExpLib::pointToPoseStamped(f.point,fa.header.frame_id),distance)){
                    //最終手段で直線距離を計算
                    distance = Eigen::Vector2d(f.point.x - p.x, f.point.y - p.y).norm();
                }                
            }
            double angle = std::abs(acos(v1.dot(v2)));
            pcr.values.emplace_back(preCalcResult::value(distance,angle));
            if(angle > mVal_.angle) mVal_.angle = std::move(angle);
            if(distance > mVal_.distance) mVal_.distance = std::move(distance);
        }
        return pcr;
    };

    ownPreCalc_.reserve(ls.size()+1);
    otherPreCalc_.reserve(ria.info.size());

    // 分岐領域の計算
    double forward = 0;
    for(const auto& l : ls){
        double distance;
        Eigen::Vector2d v1;
        if(!this->pp_.getDistanceAndVec(pose,ExpLib::pointToPoseStamped(l.point,pose.header.frame_id),distance,v1)){
            v1 = Eigen::Vector2d(l.point.x - pose.pose.position.x, l.point.y - pose.pose.position.y);
            distance = v1.lpNorm<1>();
            v1.normalize();
        }
        this->ownPreCalc_.emplace_back(calc(l.point,v1));
        forward += distance;
    }
    forward /= ls.size();

    // 直進時の計算
    Eigen::Vector2d fwdV1 = ExpLib::qToVector2d(pose.pose.orientation);; 
    this->ownPreCalc_.emplace_back(calc(ExpLib::vector2dToPoint(Eigen::Vector2d(pose.pose.position.x,pose.pose.position.y)+forward*fwdV1),fwdV1));

    // 他のロボットに関する計算
    for(const auto& ri : ria.info) this->otherPreCalc_.emplace_back(calc(ri.pose.position,ExpLib::qToVector2d(ri.pose.orientation)));
}

void SeamlessHybridExploration::simBridge(std::vector<geometry_msgs::Pose>& r, std::vector<geometry_msgs::Point>& b, std::vector<geometry_msgs::Point>& f){
    static PathPlanning<navfn::NavfnROS> pp("simulator_goal_costmap","simulator_goal_path");

    ros::NodeHandle p("~");
    std::string FRAME_ID;
    std::string EXTRA_PARAMETER_NAMESPACE;
    int SIMULATE_ROBOT_INDEX;

    p.param<std::string>("frame_id", FRAME_ID, "map");
    p.param<std::string>("extra_parameter_namespace", EXTRA_PARAMETER_NAMESPACE, "extra_parameter");
    p.param<double>("/"+EXTRA_PARAMETER_NAMESPACE+"/direction_weight", this->DIRECTION_WEIGHT, 1.5);
    p.param<double>("/"+EXTRA_PARAMETER_NAMESPACE+"/distance_weight", this->DISTANCE_WEIGHT, 2.5);
    p.param<double>("/"+EXTRA_PARAMETER_NAMESPACE+"/other_robot_weight", this->OTHER_ROBOT_WEIGHT, 1.0); 
    p.param<int>("/"+EXTRA_PARAMETER_NAMESPACE+"/simulate_robot_index", SIMULATE_ROBOT_INDEX, 1); 

    if(SIMULATE_ROBOT_INDEX > r.size()) SIMULATE_ROBOT_INDEX = 1;

    geometry_msgs::PoseStamped ps;
    std::vector<ExpLib::listStruct> ls;
    exploration_msgs::FrontierArray fa;
    exploration_msgs::RobotInfoArray ria;

    // ls 格納
    ls.resize(b.size());
    for(int i=0,ie=b.size();i!=ie;++i) ls[i].point = b[i];
    this->ls_ = ls;

    // fa 格納
    fa.frontiers.resize(f.size());
    fa.header.frame_id = FRAME_ID;
    for(int i=0,ie=f.size();i!=ie;++i) fa.frontiers[i].point = f[i];
    this->fa_ = fa;

    // ps 格納
    ps.header.frame_id = FRAME_ID;
    ps.pose = r[SIMULATE_ROBOT_INDEX-1];
    this->ps_ = ps;

    // ria 格納 
    ria.info.resize(r.size()-1);
    for(int i=0,ie=r.size(),ix=0;i!=ie;++i) {
        if(i+1 == SIMULATE_ROBOT_INDEX) continue;
        ria.info[ix++].pose = r[i];
    }
    this->ria_ = ria;

    geometry_msgs::PointStamped goal;
    this->decideGoal(goal);

    pp.createPath(ps_,ExpLib::pointStampedToPoseStamped(goal));
}

#endif // SEAMLESS_HYBRID_EXPLORATION_HPP