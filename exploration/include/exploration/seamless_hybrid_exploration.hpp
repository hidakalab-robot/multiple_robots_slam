#ifndef SEAMLESS_HYBRID_EXPLORATION_HPP
#define SEAMLESS_HYBRID_EXPLORATION_HPP

#include <exploration/sensor_based_exploration.hpp>
#include <exploration_libraly/path_planning.hpp>
#include <navfn/navfn_ros.h>
#include <exploration_msgs/RobotInfoArray.h>
#include <exploration_msgs/FrontierArray.h>

class SeamlessHybridExploration :public SensorBasedExploration
{
private:
    PathPlanning<navfn::NavfnROS> pp;
    ExpLib::subStruct<exploration_msgs::RobotInfoArray> robotArray_;
    ExpLib::subStruct<exploration_msgs::FrontierArray> frontier_;

    bool decideGoal(geometry_msgs::PointStamped& goal, std::vector<ExpLib::listStruct>& ls, const geometry_msgs::PoseStamped& pose);
    bool dataFilter(std::vector<ExpLib::listStruct>& ls);
public:
    SeamlessHybridExploration::SeamlessHybridExploration();

};

SeamlessHybridExploration::SeamlessHybridExploration()
    :robotArray_("robotArray", 1)
    ,frontier_("frontier", 1)
    ,pp("global_costmap","NavfnROS"){

}

bool SeamlessHybridExploration::decideGoal(geometry_msgs::PointStamped& goal, std::vector<ExpLib::listStruct>& ls, const geometry_msgs::PoseStamped& pose){
    if(robotArray_.q.callOne(ros::WallDuration(1))){
        return false;
    }

    if(frontier_.q.callOne(ros::WallDuration(1))){
        return false;
    }
    this->dataFilter(ls);
    return true;
}

bool SeamlessHybridExploration::dataFilter(std::vector<ExpLib::listStruct>& ls){
    //分岐領域のフィルタ
    // ROS_INFO_STREAM("through branch size : " << throughBranches.size());
    ROS_INFO_STREAM("before branches size : " << ls.size());

    // for(const auto& l : ls){
    //     //duplication filter

    //     if(l.duplication == ExpLib::DuplicationStatus::NEWER){
    //         ROS_INFO_STREAM("newer duplication!!");
    //         continue;
    //     }
    //     branches.emplace_back(i.point);
    // }

    // 分岐領域のフィルタ
    auto removeResult = std::remove_if(ls.begin(),ls.end(),[this](ExpLib::listStruct& l){return l.duplication == ExpLib::DuplicationStatus::NEWER;});
	ls.erase(std::move(removeResult),ls.end());
    
    ROS_INFO_STREAM("after branches size : " << ls.size());

    if(ls.size()==0) return false;

    //フロンティア領域のフィルタ

    ROS_INFO_STREAM("before frontiers size : " << inputFrontiers.size());

    frontiers.reserve(inputFrontiers.size());

    for(const auto& i : inputFrontiers){
        if(i.variance.x>i.variance.y ? i.variance.x : i.variance.y > VARIANCE_THRESHOLD || std::abs(i.covariance) > COVARIANCE_THRESHOLD) frontiers.emplace_back(i.point);
    }

    ROS_INFO_STREAM("after frontiers size : " << frontiers.size());

    if(frontiers.size()==0) return false;

    //ロボットリストのフィルタ
    if(!robotArray_->q.callOne(ros::WallDuration(1))){
        ROS_INFO_STREAM("before robotList size : " << robotArray_->data.info.size());
        for(const auto& r : robotArray_->data.info){
            if(!(r.name == "/" + ROBOT_NAME)) robotList.emplace_back(r);
        }
        ROS_INFO_STREAM("after robotList size : " << robotList.size());
        return true;
    }
    
    return false;
}

#endif // SEAMLESS_HYBRID_EXPLORATION_HPP