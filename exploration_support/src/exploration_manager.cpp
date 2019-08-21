#include <exploration_support/exploration_manager.hpp>

int main(int argc, char* argv[]){
    ros::init(argc,argv,"exploration_manager");
    ExplorationManager em;
    em.multiThreadMain();
    return 0;
}