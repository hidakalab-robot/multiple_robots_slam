#include <exploration_support/exploration_manager.hpp>

int main(int argc, char* argv[]){
    ros::init(argc,argv,"exploration_manager");
    ExplorationManager em;
    ros::spin();
    return 0;
}