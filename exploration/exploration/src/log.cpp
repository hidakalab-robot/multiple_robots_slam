#include <exploration/log.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pose_log_publisher");

    Log l;

    l.poseLogInitialize();

    while(ros::ok()){
        l.publishPoseLog();
    }
    
    return 0;
}