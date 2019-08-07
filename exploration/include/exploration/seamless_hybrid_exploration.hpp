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
        double distance = -DBL_MAX;
        double angle = -DBL_MAX;
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

    maxValue mVal;

    double COVARIANCE_THRESHOLD;
    double VARIANCE_THRESHOLD;
    double DIRECTION_WEIGHT;
    double DISTANCE_WEIGHT;
    double OTHER_ROBOT_WEIGHT;
    std::string ROBOT_NAME;

    PathPlanning<navfn::NavfnROS> pp;
    ExpLib::subStruct<exploration_msgs::RobotInfoArray> robotArray_;
    ExpLib::subStruct<exploration_msgs::FrontierArray> frontier_;

    std::vector<preCalcResult> ownPreCalc;
    std::vector<preCalcResult> otherPreCalc;

    bool decideGoal(geometry_msgs::PointStamped& goal, const std::vector<ExpLib::listStruct>& ls, const geometry_msgs::PoseStamped& pose);
    bool filter(std::vector<ExpLib::listStruct>& ls, exploration_msgs::FrontierArray& fa, exploration_msgs::RobotInfoArray& ria);

    void preCalc(const std::vector<ExpLib::listStruct>& ls, const exploration_msgs::FrontierArray& fa, const exploration_msgs::RobotInfoArray& ria, const geometry_msgs::PoseStamped& pose);
public:
    SeamlessHybridExploration();
};

SeamlessHybridExploration::SeamlessHybridExploration()
    :robotArray_("robotArray", 1)
    ,frontier_("frontier", 1)
    ,pp("global_costmap","NavfnROS"){

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

    std::vector<ExpLib::listStruct> lls = ls;
    exploration_msgs::FrontierArray fa = this->frontier_.data;
    exploration_msgs::RobotInfoArray ria = this->robotArray_.data;

    if(!this->filter(lls, fa, ria)) return false;
    preCalc(lls, fa, ria, pose);

    // 評価計算
    auto evaluation = [this](const double d, const double a){return DISTANCE_WEIGHT * d / mVal.distance + DIRECTION_WEIGHT * a / mVal.angle;};

    double minE = DBL_MAX;

    for(int m=0,me=ownPreCalc.size();m!=me;++m){
        double e = 1;
        for(int i=0,ie=ownPreCalc[m].values.size();i!=ie;++i){
            double subE = 0;
            if(otherPreCalc.size()==0) subE = DBL_MAX;
            else for(const auto& opc : otherPreCalc) subE += evaluation(opc.values[i].distance, opc.values[i].angle);
            e *= evaluation(ownPreCalc[m].values[i].distance, ownPreCalc[m].values[i].angle) + (OTHER_ROBOT_WEIGHT/subE);
        }
        // ROS_DEBUG_STREAM("position : (" << mainRobotInfo[m].robot.coordinate.x << "," << mainRobotInfo[m].robot.coordinate.y << "), sum : " << e);
        ROS_DEBUG_STREAM("position : (" << ownPreCalc[m].point.x << "," << ownPreCalc[m].point.y << "), sum : " << e);
        if(e < minE){
            minE = std::move(e);
            // goal = mainRobotInfo[m].robot.coordinate;
            goal.header.frame_id = pose.header.frame_id;
            goal.header.stamp = ros::Time::now();
            goal.point = ownPreCalc[m].point;
            goal_.pub.publish(goal);
            if(m == me -1) return false;
        }
    }
    lastGoal = goal.point;
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
    auto calc = [&,this](const geometry_msgs::Point& p,const Eigen::Vector2d& v1){
        SeamlessHybridExploration::preCalcResult pcr;
        pcr.point = p;
        // Eigen::Vector2d v1 = ExpLib::qToVector2d(ps.orientation);
        pcr.values.reserve(fa.frontiers.size());
        for(const auto& f : fa.frontiers){
            // 目標地点での向きをpathの最後の方の移動で決めたい
            Eigen::Vector2d v2;
            double distance;
            if(!pp.getDistanceAndVec(ExpLib::pointToPoseStamped(p,pose.header.frame_id),ExpLib::pointToPoseStamped(f.point,fa.header.frame_id),distance,v2)){
                v2 = Eigen::Vector2d(f.point.x - p.x, f.point.y - p.y).normalized();
                if(!pp.getDistance(ExpLib::pointToPoseStamped(p,pose.header.frame_id),ExpLib::pointToPoseStamped(f.point,fa.header.frame_id),distance)){
                    //最終手段で直線距離を計算
                    distance = Eigen::Vector2d(f.point.x - p.x, f.point.y - p.y).norm();
                }                
            }
            double angle = std::abs(acos(v1.dot(v2)));
            pcr.values.emplace_back(preCalcResult::value(distance,angle));
            if(angle > mVal.angle) mVal.angle = std::move(angle);
            if(distance > mVal.distance) mVal.distance = std::move(distance);
        }
        return pcr;
    };

    ownPreCalc.reserve(ls.size()+1);
    otherPreCalc.reserve(ria.info.size());

    // 分岐領域の計算
    double forward = 0;
    for(const auto& l : ls){
        double distance;
        Eigen::Vector2d v1;
        if(!pp.getDistanceAndVec(pose,ExpLib::pointToPoseStamped(l.point,pose.header.frame_id),distance,v1)){
            v1 = Eigen::Vector2d(l.point.x - pose.pose.position.x, l.point.y - pose.pose.position.y);
            distance = v1.lpNorm<1>();
            v1.normalize();
        }
        ownPreCalc.emplace_back(calc(l.point,v1));
        forward += distance;
    }
    forward /= ls.size();

    // 直進時の計算
    Eigen::Vector2d fwdV1 = ExpLib::qToVector2d(pose.pose.orientation);; 
    this->ownPreCalc.emplace_back(calc(ExpLib::vector2dToPoint(Eigen::Vector2d(pose.pose.position.x,pose.pose.position.y)+forward*fwdV1),fwdV1));

    // 他のロボットに関する計算
    for(const auto& ri : ria.info) otherPreCalc.emplace_back(calc(ri.pose.position,ExpLib::qToVector2d(ri.pose.orientation)));
}
#endif // SEAMLESS_HYBRID_EXPLORATION_HPP