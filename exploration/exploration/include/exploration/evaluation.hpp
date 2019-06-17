#ifndef EVALUATION_HPP
#define EVALUATION_HPP

#include <ros/ros.h>
#include <exploration_msgs/Frontier.h>
#include <geometry_msgs/Point.h>
#include <exploration/common_lib.hpp>
#include <exploration/path_planning.hpp>
#include <navfn/navfn_ros.h>

class Evaluation
{
private:
    struct sumValue{
        double sumDistnance;
        double sumAngle;
        geometry_msgs::Point coordinate;
    };

    struct maxValue{
        double distance;
        double angle;
        maxValue(double d, double a):distance(d),angle(a){};
    };

    double ANGLE_WEIGHT;
    double NORM_WEIGHT;
    std::string MAP_FRAME_ID;

    std::vector<exploration_msgs::Frontier> frontiers;
    std::vector<geometry_msgs::Point> branches;
    geometry_msgs::Pose pose;

    std::vector<sumValue> sVal;
    maxValue mVal;

public:
    Evaluation(const std::vector<exploration_msgs::Frontier>& f, const std::vector<geometry_msgs::Point>& b, const geometry_msgs::Pose& p);
    
    void initialize(void);
    bool result(geometry_msgs::Point& goal);// 分岐に行く場合はtrueと座標を返して、直進する場合はfalseを返す　
};

Evaluation::Evaluation(const std::vector<exploration_msgs::Frontier>& f, const std::vector<geometry_msgs::Point>& b, const geometry_msgs::Pose& p)
    :frontiers(f)
    ,branches(b)
    ,pose(p)
    ,mVal(-DBL_MAX,-DBL_MAX){

    ros::NodeHandle ph("~");
    ph.param<double>("angle_weight", ANGLE_WEIGHT, 1.5);
    ph.param<double>("norm_weight", NORM_WEIGHT, 2.5); 
    ph.param<std::string>("map_frame_id", MAP_FRAME_ID, "map"); 

    sVal.reserve(branches.size()+1);
};

void Evaluation::initialize(void){

    static PathPlanning<navfn::NavfnROS> pp("global_costmap","NavfnROS");
// ///stampのフレームid
//     for(const auto& g : goals) ROS_INFO_STREAM("path length : " << pp.getPathLength(pose_.data,CommonLib::pointToPoseStamped(g,MAP_FRAME_ID)));
    auto calc = [this](const geometry_msgs::Point& p, const Eigen::Vector2d& v1){
        ROS_DEBUG_STREAM("calc p : (" << p.x << "," << p.y << ")");
        sumValue s{0,0,p};
        for(const auto& f : frontiers){
            Eigen::Vector2d v2 = Eigen::Vector2d(f.coordinate.x - p.x, f.coordinate.y - p.y);

            double angle = std::abs(acos(v1.normalized().dot(v2.normalized())));
            // double distance = v2.lpNorm<1>();
            double distance = pp.getPathLength(CommonLib::pointToPoseStamped(p,MAP_FRAME_ID),CommonLib::pointToPoseStamped(f.coordinate,MAP_FRAME_ID));

            ROS_INFO_STREAM("legacy distance : " << v2.lpNorm<1>() << ", new distance : " << distance);

            s.sumAngle += angle;
            s.sumDistnance += distance;

            if(angle > mVal.angle) mVal.angle = std::move(angle);
            if(distance > mVal.distance) mVal.distance = std::move(distance);
        }
        return s;
    };

    double forward = 0;
    for(const auto& b : branches){
        Eigen::Vector2d v1 = Eigen::Vector2d(b.x - pose.position.x, b.y - pose.position.y);
        sVal.emplace_back(calc(b,v1));
        forward += v1.norm();
    }
    forward /= branches.size();

    //直進時の計算
    double yaw = CommonLib::qToYaw(pose.orientation);
    double cosYaw = cos(yaw);
    double sinYaw = sin(yaw);

    sVal.emplace_back(calc(CommonLib::msgPoint(pose.position.x+forward*cosYaw,pose.position.y+forward*sinYaw),Eigen::Vector2d(cosYaw,sinYaw)));
}

bool Evaluation::result(geometry_msgs::Point& goal){ 
    ROS_DEBUG_STREAM("sVal size : " << sVal.size());

    double minE = DBL_MAX;

    for(int i=0,ie=sVal.size();i!=ie;++i){
        double e = NORM_WEIGHT * sVal[i].sumDistnance / mVal.distance + ANGLE_WEIGHT * sVal[i].sumAngle / mVal.angle;
        ROS_DEBUG_STREAM("position : (" << sVal[i].coordinate.x << "," << sVal[i].coordinate.y << "), sum : " << e);
        if(e < minE){
            if(i == ie-1) return false;
            minE = std::move(e);
            goal = sVal[i].coordinate;
        }
    }
    return true;
}

#endif //EVALUATION_HPP