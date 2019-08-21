#include <exploration_support/branch_detection.hpp>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "branch_detection");
    BranchDetection bd;
    ros::spin();
    return 0;
}