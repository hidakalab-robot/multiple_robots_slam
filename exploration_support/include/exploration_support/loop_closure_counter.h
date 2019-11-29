#ifndef LOOP_CLOSURE_COUNTER_H
#define LOOP_CLOSURE_COUNTER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <exploration_libraly/struct.h>
#include <exploration_libraly/construct.h>
#include <dynamic_reconfigure/server.h>
#include <exploration_support/loop_closure_counter_parameter_reconfigureConfig.h>
#include <fstream>
#include <thread>

// legacy loop counter
namespace ExStc = ExpLib::Struct;
namespace ExCos = ExpLib::Construct;

class LoopClosureCounter
{
private:
    // dynamic parameters
    double LOOP_CLOSURE_THRESHOLD;

    // static parameters
    std::string ODOM_FRAME_ID;
    std::string MAP_FRAME_ID;
    double PUBLISH_RATE;
    std::string LOOP_PARAMETER_FILE_PATH;
    bool OUTPUT_LOOP_PARAMETERS;

    // variables
    ExStc::pubStruct<std_msgs::Int8> count;
    ExStc::pubStruct<std_msgs::Float64> accumTemp;
    ExStc::pubStruct<std_msgs::Float64> accumPerm;
    dynamic_reconfigure::Server<exploration_support ::loop_closure_counter_parameter_reconfigureConfig> drs_;

    // functions
    void loopDetection(void);
    void loadParams(void);
    void dynamicParamsCB(exploration_support::loop_closure_counter_parameter_reconfigureConfig &cfg, uint32_t level);
    void outputParams(void);

public:
    LoopClosureCounter();
    ~LoopClosureCounter();
    void loopDetectionLoop(void);
};

#endif // LOOP_CLOSURE_COUNTER_HPP