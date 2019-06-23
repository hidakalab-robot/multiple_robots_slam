#include <exploration/multi_exploration_simulator.hpp>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "multi_exploratino_simulator");
    MultiExplorationSimulator mes;

    dynamic_reconfigure::Server<exploration::multi_exploration_simulatorConfig> server;
    dynamic_reconfigure::Server<exploration::multi_exploration_simulatorConfig>::CallbackType cbt;

    auto fn = [](std::vector<geometry_msgs::Pose>& r, std::vector<geometry_msgs::Point>& b, std::vector<geometry_msgs::Point>& f)->void{};

    cbt = boost::bind(&MultiExplorationSimulator::callback,&mes, _1, _2, fn);
    server.setCallback(cbt);

    ros::spin();
    return 0;
}