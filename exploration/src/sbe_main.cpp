#include <ros/ros.h>
#include <exploration/moving.h>
#include <exploration/branch_search.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sensor_based_exploration");

    BranchSearch bs;
    Moving mv;

    geometry_msgs::Point goal;

    while(ros::ok()){
        if(bs.getGoal(goal)){
            mv.moveToGoal(goal);
        }
        else{
            mv.moveToForward();
        }
    }
    
    return 0;
}
