#include <dynamic_reconfigure/server.h>
#include <exploration/movement_parameter_reconfigureConfig.h>
#include <fstream>
#include <ros/ros.h>

class MovementParameter{
private:
    std::string MOVEMENT_PARAMETER_FILE_PATH;
    bool OUTPUT_MOVEMENT_PARAMETERS;

    //reconfig parameters
    double FORWARD_VELOCITY;
    double BACK_VELOCITY;
    double BACK_TIME;
    double BUMPER_ROTATION_TIME;
    double FORWARD_ANGLE;
    double ROTATION_VELOCITY;
    double EMERGENCY_THRESHOLD;
    double ROAD_CENTER_THRESHOLD;
    double ROAD_THRESHOLD;
    double CURVE_GAIN;
    double AVOIDANCE_GAIN;
    double ROAD_CENTER_GAIN;
    double WALL_FORWARD_ANGLE;
    double WALL_RATE_THRESHOLD;
    double WALL_DISTANCE_UPPER_THRESHOLD;
    double WALL_DISTANCE_LOWER_THRESHOLD;
    double EMERGENCY_DIFF_THRESHOLD;
    double ANGLE_BIAS;
    int PATH_BACK_INTERVAL;
    double GOAL_RESET_RATE;
    double COSTMAP_MARGIN;
    int ESC_MAP_DIV_X;
    int ESC_MAP_DIV_Y;
    double ESC_MAP_WIDTH_X;
    double ESC_MAP_HEIGHT_Y;
    double SAFETY_RANGE_THRESHOLD;
    double SAFETY_RATE_THRESHOLD;

    void callback(exploration::extra_parameter_reconfigureConfig &config, uint32_t level){
        FORWARD_VELOCITY = cfg.forward_velocity;
        BACK_VELOCITY = cfg.back_velocity;
        BACK_TIME = cfg.;
        BUMPER_ROTATION_TIME = cfg.;
        FORWARD_ANGLE = cfg.;
        ROTATION_VELOCITY = cfg.;
        EMERGENCY_THRESHOLD = cfg.;
        ROAD_CENTER_THRESHOLD = cfg.;
        ROAD_THRESHOLD = cfg.;
        CURVE_GAIN = cfg.;
        AVOIDANCE_GAIN = cfg.;
        ROAD_CENTER_GAIN = cfg.;
        WALL_FORWARD_ANGLE = cfg.;
        WALL_RATE_THRESHOLD = cfg.;
        WALL_DISTANCE_UPPER_THRESHOLD = cfg.;
        WALL_DISTANCE_LOWER_THRESHOLD = cfg.;
        EMERGENCY_DIFF_THRESHOLD = cfg.;
        ANGLE_BIAS = cfg.;
        PATH_BACK_INTERVAL = cfg.;
        GOAL_RESET_RATE = cfg.;
        COSTMAP_MARGIN = cfg.;
        ESC_MAP_DIV_X = cfg.;
        ESC_MAP_DIV_Y = cfg.;
        ESC_MAP_WIDTH_X = cfg.;
        ESC_MAP_HEIGHT_Y = cfg.;
        SAFETY_RANGE_THRESHOLD = cfg.;
        SAFETY_RATE_THRESHOLD = cfg.;
    };
    void outputParams(void){

    };

    // 変数名を文字列にできれば。。。

public:
    MovementParameter(){
        ros::NodeHandle nh("~");
        nh.param<std::string>("movement_parameter_file_path",MOVEMENT_PARAMETER_FILE_PATH,"movement_last_parameters.yaml");
        nh.param<bool>("output_movement_parameters",OUTPUT_MOVEMENT_PARAMETERS,true);
    };
};

int main(int argc, char* argv[]){
    ros::init(argc, argv, "movement_parameter");

    ros::NodeHandle nh("~");

    dynamic_reconfigure::Server<exploration::movement_parameter_reconfigureConfig> server;
    dynamic_reconfigure::Server<exploration::movement_parameter_reconfigureConfig>::CallbackType cbt;

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