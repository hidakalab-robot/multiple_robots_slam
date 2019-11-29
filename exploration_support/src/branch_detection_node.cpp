#include <exploration_support/branch_detection.h>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "branch_detection");
    BranchDetection bd;
    ros::spin();
    return 0;
}