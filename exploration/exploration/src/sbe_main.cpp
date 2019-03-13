#include <exploration/movement.hpp>
#include <exploration/branch_search.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sensor_based_exploration");

    BranchSearch bs;
    Movement mv;

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
