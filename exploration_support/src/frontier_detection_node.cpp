#include <exploration_support/frontier_detection.h>
#include <ros/ros.h>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "frontier_detection");
    FrontierDetection fd;
    ros::spin();
    return 0;
}