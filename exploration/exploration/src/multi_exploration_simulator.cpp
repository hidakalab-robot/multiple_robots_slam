#include <exploration/multi_exploration_simulator.hpp>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "multi_exploratino_simulator");
    MultiExplorationSimulator mes;

    dynamic_reconfigure::Server<exploration::multi_exploration_simulatorConfig> server;
    dynamic_reconfigure::Server<exploration::multi_exploration_simulatorConfig>::CallbackType cbt;

    cbt = boost::bind(&MultiExplorationSimulator::callback,&mes, _1, _2);
    server.setCallback(cbt);

    ros::spin();
    return 0;
}