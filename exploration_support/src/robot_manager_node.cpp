#include <exploration_support/robot_manager.h>
#include <ros/ros.h>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "robot_manager");
    RobotManager rm;
    rm.multiThreadMain();
    return 0;
}