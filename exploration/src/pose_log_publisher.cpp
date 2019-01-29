#include <ros/ros.h>
#include <exploration/tool.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pose_log_publisher");

    Tool tl;

    while(ros::ok()){
        tl.poseLogPublish();
    }
    
    return 0;
}