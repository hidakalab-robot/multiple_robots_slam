#include <exploration/visualization.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "visualization_test");

    Visualization v;

    //v.poseMarkerInitialize();
    v.goalMarkerInitialize();
    v.goalListMarkerInitialize();
    //v.toGoalMarkerInitialize();
    //v.moveAngleMarkerInitialize();
    v.goalDeleteInitialize();
    v.goalListDeleteInitialize();
    //v.toGoalDeleteInitialize();

    while(ros::ok()){
        //v.publishPoseMarker();
        v.publishGoalMarker();
        v.publishGoalListMarker();
        //v.publishToGoalMarker();
        //v.publishMoveAngleMarker();
        v.publishGoalDelete();
        v.publishGoalListDelete();
        //v.publishToGoalDelete();
    }
    
    return 0;
}