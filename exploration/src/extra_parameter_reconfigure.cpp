#include <dynamic_reconfigure/server.h>
#include <exploration/extra_parameter_reconfigureConfig.h>
#include <fstream>
#include <ros/ros.h>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "extra_parameter");

    ros::NodeHandle nh("~");
    std::string EXTRA_PARAMETER_FILE_PATH;
    bool OUTPUT_EXTRA_PARAMETERS;

    nh.param<std::string>("extra_parameter_file_path",EXTRA_PARAMETER_FILE_PATH,"simulator_last_extra_parameters.yaml");
    nh.param<bool>("output_extra_parameters",OUTPUT_EXTRA_PARAMETERS,true);

    dynamic_reconfigure::Server<exploration::extra_parameter_reconfigureConfig> server;
    dynamic_reconfigure::Server<exploration::extra_parameter_reconfigureConfig>::CallbackType cbt;

    int SIMULATE_ROBOT_INDEX;
    double DIRECTION_WEIGHT,DISTANCE_WEIGHT,OTHER_ROBOT_WEIGHT;

    cbt = boost::bind(+[](exploration::extra_parameter_reconfigureConfig &config, uint32_t level, int* sri, double* dr, double* ds, double* orw)->void{
        *sri = config.simulate_robot_index;
        *dr = config.direction_weight;
        *ds = config.distance_weight;
        *orw = config.other_robot_weight;
    }, _1, _2, &SIMULATE_ROBOT_INDEX, &DIRECTION_WEIGHT, &DIRECTION_WEIGHT, &OTHER_ROBOT_WEIGHT);

    server.setCallback(cbt);
    ros::spin();
    
    if(OUTPUT_EXTRA_PARAMETERS){
        std::cout << "writing last parameters ... ..." << std::endl;
        std::ofstream ofs(EXTRA_PARAMETER_FILE_PATH);
    
        if(ofs) std::cout << "file open succeeded" << std::endl;
        else {
            std::cout << "file open failed" << std::endl;
            return 0 ;
        }
        ofs << "simulate_robot_index: " << SIMULATE_ROBOT_INDEX << std::endl;
        ofs << "direction_weight: " << DIRECTION_WEIGHT << std::endl;
        ofs << "distance_weight: " << DISTANCE_WEIGHT << std::endl;
        ofs << "other_robot_weight: " << OTHER_ROBOT_WEIGHT << std::endl;
    }

    return 0;
}