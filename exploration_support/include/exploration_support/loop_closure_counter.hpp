#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <exploration_libraly/struct.hpp>
#include <exploration_libraly/construct.hpp>
#include <dynamic_reconfigure/server.h>
#include <exploration_support/loop_closure_counter_parameter_reconfigureConfig.h>
#include <fstream>

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

    

    void loadParams(void);
    void dynamicParamsCB(exploration_support::loop_closure_counter_parameter_reconfigureConfig &cfg, uint32_t level);
    void outputParams(void);

public:
    LoopClosureCounter();
    ~LoopClosureCounter(){if(OUTPUT_LOOP_PARAMETERS) outputParams();};
    void loopClosureDetection(void);
};

LoopClosureCounter::LoopClosureCounter()
    :count("loop_closure_counter/count",1)
    ,accumTemp("loop_closure_counter/temp_accumlate",1)
    ,accumPerm("loop_closure_counter/perm_accumlate",1)
    ,drs_(ros::NodeHandle("~/loop")){
    loadParams();
    drs_.setCallback(boost::bind(&LoopClosureCounter::dynamicParamsCB,this, _1, _2));{
}

void LoopClosureCounter::loopClosureDetection(void){

}

void LoopClosureCounter::loadParams(void){
    ros::NodeHandle nh("~/loop");
    // dynamic parameters
    nh.param<double>("loop_closure_threshold",LOOP_CLOSURE_THRESHOLD,0.0);
    // static parameters
    nh.param<std::string>("odom_frame_id",ODOM_FRAME_ID,"odom");
    nh.param<std::string>("map_frame_id",MAP_FRAME_ID,"map");
    nh.param<double>("publish_rate",PUBLISH_RATE,10.0);
    nh.param<std::string>("loop_parameter_file_path",LOOP_PARAMETER_FILE_PATH,"loop_last_parameters.yaml");
    nh.param<bool>("output_loop_parameters",OUTPUT_LOOP_PARAMETERS,true);
}

void LoopClosureCounter::dynamicParamsCB(exploration_support::loop_closure_counter_parameter_reconfigureConfig &cfg, uint32_t level){
    LOOP_CLOSURE_THRESHOLD = cfg.loop_closure_threshold;
}

void LoopClosureCounter::outputParams(void){
    std::cout << "writing loop last parameters ... ..." << std::endl;
    std::ofstream ofs(LOOP_CLOSURE_THRESHOLD);

    if(ofs) std::cout << "loop param file open succeeded" << std::endl;
    else {
        std::cout << "loop param file open failed" << std::endl;
        return 0;
    }

    ofs << "loop_closure_threshold: " << LOOP_CLOSURE_THRESHOLD << std::endl;
}