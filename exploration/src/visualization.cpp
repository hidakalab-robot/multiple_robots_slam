#include <exploration/visualization.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "visualization");

    Visualization v;

    v.poseMarkerInitialize();
    v.goalMarkerInitialize();
    v.goalListMarkerInitialize();
    v.toGoalMarkerInitialize();
    v.moveAngleMarkerInitialize();

    while(ros::ok()){
        v.publishPoseMarker();
        v.publishGoalMarker();
        v.publishGoalListMarker();
        v.publishToGoalMarker();
        v.publishMoveAngleMarker();
    }
    
    return 0;
}