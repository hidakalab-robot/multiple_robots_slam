#include <exploration/visualization.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "visualization_fbe");

    Visualization v;

    v.poseMarkerInitialize();
    v.goalMarkerInitialize();
    v.goalListMarkerInitialize();
    v.goalDeleteInitialize();
    v.goalListDeleteInitialize();

    while(ros::ok()){
        v.publishGoalDelete();
        v.publishGoalListDelete();
        v.publishPoseMarker();
        v.publishGoalMarker();
        v.publishGoalListMarker();
    }
    
    return 0;
}