#ifndef SENSOR_BASED_EXPLORATION_HPP
#define SENSOR_BASED_EXPLORATION_HPP

#include <ros/ros.h>
#include <exploration_libraly/struct.h>
#include <exploration_msgs/PointArray.h>
#include <exploration_msgs/PoseStampedArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <exploration/sensor_based_exploration_parameter_reconfigureConfig.h>

namespace ExStc = ExpLib::Struct;

class SensorBasedExploration
{
private:
    // static parameters
    bool OUTPUT_SBE_PARAMETERS;
    std::string SBE_PARAMETER_FILE_PATH;

    // variables
    ExStc::subStruct<exploration_msgs::PointArray> branch_;
    ExStc::subStruct<geometry_msgs::PoseStamped> pose_;
    ExStc::subStruct<exploration_msgs::PoseStampedArray> poseLog_;

    // functions
    void duplicateDetection(std::vector<ExStc::listStruct>& ls, const exploration_msgs::PoseStampedArray& log);
    virtual bool decideGoal(geometry_msgs::PointStamped& goal, const std::vector<ExStc::listStruct>& ls, const geometry_msgs::PoseStamped& pose);
    virtual void loadParams(void);
    virtual void dynamicParamsCB(exploration::sensor_based_exploration_parameter_reconfigureConfig &cfg, uint32_t level);
    virtual void outputParams(void);

protected:
    // dynamic parameters
    double LAST_GOAL_TOLERANCE;
    double DUPLICATE_TOLERANCE;
    double LOG_CURRENT_TIME;//if 30 -> 30秒前までのログで重複検出
    double NEWER_DUPLICATION_THRESHOLD;//最近通った場所の重複とみなす時間の上限,時間の仕様はLOG_NEWER_LIMITと同じ
    
    // variables
    ExStc::pubStruct<geometry_msgs::PointStamped> goal_;
    dynamic_reconfigure::Server<exploration::sensor_based_exploration_parameter_reconfigureConfig> drs_;
    geometry_msgs::Point lastGoal_;

public:
    SensorBasedExploration();
    virtual ~SensorBasedExploration();
    bool getGoal(geometry_msgs::PointStamped& goal);
};

#endif // SENSOR_BASED_EXPLORATION_HPP