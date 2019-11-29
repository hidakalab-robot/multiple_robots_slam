#ifndef FRONTIER_BASED_EXPLORATION_HPP
#define FRONTIER_BASED_EXPLORATION_HPP

#include <ros/ros.h>
#include <exploration_libraly/struct.hpp>
#include <exploration_msgs/FrontierArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <exploration/frontier_based_exploration_parameter_reconfigureConfig.h>

namespace ExStc = ExpLib::Struct;

class FrontierBasedExploration
{
private:
    // dynamic parameters
    double DISTANCE_WEIGHT;
    double DIRECTION_WEIGHT;
    bool LAST_GOAL_EFFECT;
    double LAST_GOAL_TOLERANCE;

    // static parameters
    std::string FBE_PARAMETER_FILE_PATH;
    bool OUTPUT_FBE_PARAMETERS;

    // variables
    ExStc::subStruct<exploration_msgs::FrontierArray> frontier_;
    ExStc::subStruct<geometry_msgs::PoseStamped> pose_;
    ExStc::pubStruct<geometry_msgs::PointStamped> goal_;
    dynamic_reconfigure::Server<exploration::frontier_based_exploration_parameter_reconfigureConfig> drs_;
    geometry_msgs::Point lastGoal_;

    // functions
    bool decideGoal(geometry_msgs::PointStamped& goal, const std::vector<exploration_msgs::Frontier>& frontiers, const geometry_msgs::PoseStamped& pose);
    void loadParams(void);
    void dynamicParamsCB(exploration::frontier_based_exploration_parameter_reconfigureConfig &cfg, uint32_t level);
    void outputParams(void);

public:
    FrontierBasedExploration();
    ~FrontierBasedExploration(){if(OUTPUT_FBE_PARAMETERS) outputParams();};
    bool getGoal(geometry_msgs::PointStamped& goal);
};

#endif // FRONTIER_BASED_EXPLORATION_HPP