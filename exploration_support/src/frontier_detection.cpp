#include <exploration_support/frontier_detection.hpp>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "frontier_detection");
    FrontierDetection fd;
    ros::spin();
    return 0;
}