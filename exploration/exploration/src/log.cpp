#include <exploration/log.hpp>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "pose_log_publisher");

    Log l;

    while(ros::ok()){
        l.publishPoseLog();
    }
    
    return 0;
}