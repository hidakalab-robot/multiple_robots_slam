#ifndef BRANCH_DETECTION_H
#define BRANCH_DETECTION_H

#include <exploration_libraly/construct.h>
#include <exploration_libraly/struct.h>
#include <exploration_libraly/utility.h>
#include <exploration_msgs/PointArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <exploration_support/branch_detection_parameter_reconfigureConfig.h>
#include <fstream>

namespace ExStc = ExpLib::Struct;
namespace ExUtl = ExpLib::Utility;
namespace ExCos = ExpLib::Construct;

class BranchDetection
{
private:
    // dynamic parameters
    double OBSTACLE_CHECK_ANGLE;
    double OBSTACLE_RANGE_THRESHOLD;
    double BRANCH_RANGE_THRESHOLD;
	double BRANCH_DIFF_X_MIN;
    double BRANCH_DIFF_X_MAX;
	double BRANCH_DIFF_Y_MIN;
    double BRANCH_DIFF_Y_MAX;

    // static parameters
    std::string BRANCH_PARAMETER_FILE_PATH;
    bool OUTPUT_BRANCH_PARAMETERS;

    // variables
    ExStc::subStructSimple scan_;
    ExStc::subStruct<geometry_msgs::PoseStamped> pose_;
    ExStc::pubStruct<exploration_msgs::PointArray> branch_;
    dynamic_reconfigure::Server<exploration_support::branch_detection_parameter_reconfigureConfig> drs_;

    // functions
    void scanCB(const sensor_msgs::LaserScan::ConstPtr& msg);
    void publishBranch(const std::vector<geometry_msgs::Point>& branches, const std::string& frameId);
    void loadParams(void);
    void dynamicParamsCB(exploration_support::branch_detection_parameter_reconfigureConfig &cfg, uint32_t level);
    void outputParams(void);

public:
    BranchDetection();
    ~BranchDetection();
};

#endif // BRANCH_DETECTION_H