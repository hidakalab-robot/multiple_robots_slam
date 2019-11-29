//探査の終了条件を判断する
#ifndef EXPLORATION_MANAGER_H
#define EXPLORATION_MANAGER_H

#include <exploration_libraly/struct.h>
#include <exploration_libraly/construct.h>
#include <exploration_msgs/FrontierArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <thread>
#include <dynamic_reconfigure/server.h>
#include <exploration_support/exploration_manager_parameter_reconfigureConfig.h>
#include <fstream>

namespace ExStc = ExpLib::Struct;
namespace ExCos = ExpLib::Construct;

class ExplorationManager
{
private:
    // dynamic parameters
    double END_AREA;
    int END_FRONTIER;
    double END_TIME;

    // static parameters
    std::string EXMNG_PARAMETER_FILE_PATH;
    bool OUTPUT_EXMNG_PARAMETERS;

    // variables
    ExStc::subStructSimple map_;
    ExStc::subStructSimple frontier_;
    ExStc::pubStruct<std_msgs::Bool> areaEnd_;
    ExStc::pubStruct<std_msgs::Bool> frontierEnd_;
    ExStc::pubStruct<std_msgs::Bool> timerEnd_;
    ExStc::pubStruct<std_msgs::Float64> areaVal_;
    ExStc::pubStruct<std_msgs::Int32> frontierVal_;
    ExStc::pubStruct<std_msgs::Float64> timerVal_;
    dynamic_reconfigure::Server<exploration_support::exploration_manager_parameter_reconfigureConfig> drs_;

    // functions
    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void frontierCB(const exploration_msgs::FrontierArray::ConstPtr& msg);
    void timer(void);
    void loadParams(void);
    void dynamicParamsCB(exploration_support::exploration_manager_parameter_reconfigureConfig &cfg, uint32_t level);
    void outputParams(void);

public:
    ExplorationManager();
    ~ExplorationManager();
    void multiThreadMain(void);
};

#endif //EXPLORATION_MANAGER_H