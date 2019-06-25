#include <ros/ros.h>
#include <exploration/my_evaluationConfig.h>
#include <dynamic_reconfigure/server.h>

void callback(exploration::my_evaluationConfig &config, uint32_t level){}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "my_evaluation_reconfigure");

    dynamic_reconfigure::Server<exploration::my_evaluationConfig> server;
    dynamic_reconfigure::Server<exploration::my_evaluationConfig>::CallbackType cbt;

    cbt = boost::bind(&callback, _1, _2);
    server.setCallback(cbt);
    ros::spin();
    return 0;
}