#include <ros/ros.h>
#include <exploration/extra_parameter_reconfigureConfig.h>
#include <dynamic_reconfigure/server.h>
#include <fstream>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "my_evaluation_reconfigure");

    ros::NodeHandle nh("~");
    std::string EXTRA_PARAMETER_FILE_PATH;
    bool OUTPUT_EXTRA_PARAMETERS;

    nh.param<std::string>("extra_parameter_file_path",EXTRA_PARAMETER_FILE_PATH,"simulator_last_extra_parameters.yaml");
    nh.param<bool>("output_extra_parameters",OUTPUT_EXTRA_PARAMETERS,true);

    dynamic_reconfigure::Server<exploration::extra_parameter_reconfigureConfig> server;
    dynamic_reconfigure::Server<exploration::extra_parameter_reconfigureConfig>::CallbackType cbt;

    int SIMULATE_ROBOT_INDEX;
    double ANGLE_WEIGHT,PATH_WEIGHT,ROBOT_WEIGHT;

    cbt = boost::bind(+[](exploration::extra_parameter_reconfigureConfig &config, uint32_t level, int* sri, double* aw, double* pw, double* rw)->void{
        *sri = config.simulate_robot_index;
        *aw = config.angle_weight;
        *pw = config.path_weight;
        *rw = config.robot_weight;
    }, _1, _2, &SIMULATE_ROBOT_INDEX, &ANGLE_WEIGHT, &PATH_WEIGHT, &ROBOT_WEIGHT);

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
        ofs << "angle_weight: " << ANGLE_WEIGHT << std::endl;
        ofs << "path_weight: " << PATH_WEIGHT << std::endl;
        ofs << "robot_weight: " << ROBOT_WEIGHT << std::endl;
    }

    return 0;
}