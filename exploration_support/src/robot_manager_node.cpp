#include <exploration_support/robot_manager.h>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "robot_manager");
    RobotManager rm;
    rm.multiThreadMain();
    return 0;
}