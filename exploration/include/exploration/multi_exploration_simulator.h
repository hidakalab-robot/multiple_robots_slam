#ifndef MULTI_EXPLORATION_SIMULATOR_HPP
#define MULTI_EXPLORATION_SIMULATOR_HPP

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <exploration/multi_exploration_simulatorConfig.h>
#include <exploration_libraly/struct.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

namespace ExStc = ExpLib::Struct;

class MultiExplorationSimulator
{
private:
    // dynamic parameters
    int ROBOT_NUMBER;
    int BRANCH_NUMBER;
    int FRONTIER_NUMBER;

    // static parameters
    std::string MAP_FRAME_ID;
    double BRANCH_SCALE;
    double FRONTIER_SCALE;
    std::string MULSIM_PARAMETER_FILE_PATH;
    bool OUTPUT_MULSIM_PARAMETERS;

    // variables
    ExStc::pubStruct<geometry_msgs::PoseArray> poses_;
    ExStc::pubStruct<visualization_msgs::Marker> branches_;
    ExStc::pubStruct<visualization_msgs::Marker> frontiers_;
    ros::NodeHandle nh_;
    dynamic_reconfigure::Server<exploration::multi_exploration_simulatorConfig> drs_;
    geometry_msgs::PoseArray robotPoses_;
    visualization_msgs::Marker branchCoordinates_;
    visualization_msgs::Marker frontierCoordinates_;

    // functions
    void loadParams(void);
    void dynamicParamsCB(exploration::multi_exploration_simulatorConfig &cfg, uint32_t level);
    void outputParams(void);

public:
    MultiExplorationSimulator();
    // ~MultiExplorationSimulator(){if(OUTPUT_MULSIM_PARAMETERS) outputParams();};
    ~MultiExplorationSimulator();
    void updateParams(std::function<void(std::vector<geometry_msgs::Pose>&, std::vector<geometry_msgs::Point>&, std::vector<geometry_msgs::Point>&)> fn);
};

#endif // MULTI_EXPLORATION_SIMULATOR_HPP