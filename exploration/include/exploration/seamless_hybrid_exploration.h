#ifndef SEAMLESS_HYBRID_EXPLORATION_HPP
#define SEAMLESS_HYBRID_EXPLORATION_HPP

#include <ros/ros.h>
#include <exploration/sensor_based_exploration.h>
#include <exploration_libraly/struct.h>
#include <exploration_libraly/path_planning.h>
#include <exploration_msgs/FrontierArray.h>
#include <exploration_msgs/RobotInfoArray.h>
#include <navfn/navfn_ros.h>
#include <dynamic_reconfigure/server.h>
#include <exploration/seamless_hybrid_exploration_parameter_reconfigureConfig.h>

namespace ExStc = ExpLib::Struct;

class SeamlessHybridExploration :public SensorBasedExploration
{
private:
    // dynamic parameters
    double DISTANCE_WEIGHT;
    double DIRECTION_WEIGHT;
    double VARIANCE_THRESHOLD;
    double COVARIANCE_THRESHOLD;
    double OTHER_ROBOT_WEIGHT;

    // static parameters
    std::string ROBOT_NAME;
    std::string SHE_PARAMETER_FILE_PATH;
    bool OUTPUT_SHE_PARAMETERS;

    // struct
    struct maxValue{
        double distance;
        double angle;
        maxValue();
    };
    struct preCalcResult{
        struct value{
            double distance;
            double angle;
            value();
            value(const double d, const double a);
        };
        geometry_msgs::Point point;
        std::vector<value> values;
        preCalcResult();
    };

    // variables
    ExStc::subStruct<exploration_msgs::RobotInfoArray> robotArray_;
    ExStc::subStruct<exploration_msgs::FrontierArray> frontier_;
    ExStc::pubStruct<exploration_msgs::FrontierArray> useFro_;
    ExpLib::PathPlanning<navfn::NavfnROS> pp_;
    dynamic_reconfigure::Server<exploration::seamless_hybrid_exploration_parameter_reconfigureConfig> drs_;
    std::vector<ExStc::listStruct> ls_;
    exploration_msgs::FrontierArray fa_;
    exploration_msgs::RobotInfoArray ria_;
    std::vector<preCalcResult> ownPreCalc_;
    std::vector<preCalcResult> otherPreCalc_;
    geometry_msgs::PoseStamped ps_;
    maxValue mVal_;

    // functions
    bool decideGoal(geometry_msgs::PointStamped& goal);
    bool decideGoal(geometry_msgs::PointStamped& goal, const std::vector<ExStc::listStruct>& ls, const geometry_msgs::PoseStamped& pose);
    bool filter(std::vector<ExStc::listStruct>& ls, exploration_msgs::FrontierArray& fa, exploration_msgs::RobotInfoArray& ria);
    void preCalc(const std::vector<ExStc::listStruct>& ls, const exploration_msgs::FrontierArray& fa, const exploration_msgs::RobotInfoArray& ria, const geometry_msgs::PoseStamped& pose);
    void loadParams(void);
    void dynamicParamsCB(exploration::seamless_hybrid_exploration_parameter_reconfigureConfig &cfg, uint32_t level);
    void outputParams(void);

public:
    SeamlessHybridExploration();
    ~SeamlessHybridExploration();
    void simBridge(std::vector<geometry_msgs::Pose>& r, std::vector<geometry_msgs::Point>& b, std::vector<geometry_msgs::Point>& f);
};

#endif // SEAMLESS_HYBRID_EXPLORATION_HPP